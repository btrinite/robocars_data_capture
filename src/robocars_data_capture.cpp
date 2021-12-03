/**
 * @file robocars_data_capture.cpp
 * @brief Module to capture images and associated data.
 * 
 * Copyright (c) 2020 Benoit TRINITE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * Topic subscribed : 
 *  - /steering_ctrl/output : current steering order sent to actutors
 *  - /throttling_ctrl/output : current throttling order sent to ESC
 *  - /sensors/tof1 : current data from sensors, not used for now
 *  - /sensors/tof2 : current data from sensors, not used for now
 *  - /front_video_resize/image : current image from camera
 *  - /annotation/mark : mark from radio controller (for manual marking with free channels)
 * 
 * Topic published :
 *  - None 
 * Parameters :
 *  - loop_hz : main loop refresh freq
 *  - encoding : expected format for saved image 
 *  - filename_pattern : pattern to use to create file for recording
 */
#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <date.h>
#include <json.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_brain_state.h>
#include <robocars_msgs/robocars_mark.h>
#include <robocars_msgs/robocars_tof.h>
#include <robocars_msgs/robocars_telemetry.h>

#include <robocars_data_capture/robocars_datacapture_stats.h>

#include <robocars_data_capture.hpp>

RosInterface * ri;

static int loop_hz;
static std::string encoding;
static std::string filename_pattern;
static std::string base_path;
static std::string dataset_path;
static bool mark_based_filtering;
static bool throttle_based_filtering;
static boost::format file_format;
static boost::format dataset_path_format;
static float throttle_threshold;
static int metadata_shift;


const char* drivingMode2str[] = {"idle", "user", "pilot"};
static uint32_t drivingMode;

class onRunningMode;
class onIdle;
class onManualDriving;
class onAutonomousDriving;

class onRunningMode
: public RobocarsStateMachine
{
    public:
        onRunningMode() : RobocarsStateMachine("onRunningMode"),__tick_count(0) {};
        onRunningMode(const char * subStateName) : RobocarsStateMachine(subStateName),__tick_count(0) {};


    protected:

        uint32_t __tick_count;
        
        void entry(void) override {
            RobocarsStateMachine::entry();
        };

        void react(ManualDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( AutonomousDrivingEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react( IdleStatusEvent const & e) override { 
            RobocarsStateMachine::react(e);
        };

        void react(TickEvent const & e) override {
            RobocarsStateMachine::react(e);
            __tick_count++;
            if ((__tick_count%(uint32_t)(2000/loop_hz))==0) {
                ri->reportStats();
            }
        };

};

class onIdle
: public onRunningMode
{
    public:
        onIdle() : onRunningMode("onArm") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
            drivingMode = 0;
        };
  
        void react(ManualDrivingEvent const & e) override { 
            onRunningMode::react(e);
            transit<onManualDriving>();
        };


        void react(TickEvent const & e) override {
            onRunningMode::react(e);
        };

};

class onManualDriving
: public onRunningMode
{
    public:
        onManualDriving() : onRunningMode("onManualDriving") {};

    private:

        void entry(void) override {
            onRunningMode::entry();
            drivingMode=1;
            ri->newDataSet(drivingMode);
            ri->enableCapture();
        };

        void react (AutonomousDrivingEvent const & e) override {
            onRunningMode::react(e);
            ri->disableCapture();
            transit<onAutonomousDriving>();
        }

        void react(IdleStatusEvent const & e) override { 
            onRunningMode::react(e);
            ri->disableCapture();
            transit<onIdle>();
        };

        void react (RadioChannelEvent const & e) override {
        }

        void react (TickEvent const & e) override {
            onRunningMode::react(e);
        };

};

class onAutonomousDriving
: public onRunningMode
{
    public:
        onAutonomousDriving() : onRunningMode("onAutonomousDriving") {};

    private:

        virtual void entry(void) { 
            onRunningMode::entry();
            drivingMode=2;
            ri->newDataSet(drivingMode);
            ri->enableCapture();
        };  

        virtual void react(IdleStatusEvent                 const & e) override { 
            onRunningMode::react(e);
            ri->disableCapture();
            transit<onIdle>();
        };

        virtual void react(ManualDrivingEvent              const & e) override { 
            onRunningMode::react(e);
            ri->disableCapture();
            transit<onManualDriving>();
        };

        virtual void react(TickEvent                      const & e) override { 
            onRunningMode::react(e);
        };
};

FSM_INITIAL_STATE(RobocarsStateMachine, onIdle)

uint32_t mapRange(uint32_t in1,uint32_t in2,uint32_t out1,uint32_t out2,uint32_t value)
{
  if (value<in1) {value=in1;}
  if (value>in2) {value=in2;}
  return out1 + ((value-in1)*(out2-out1))/(in2-in1);
}

void RosInterface::initParam() {
    if (!node_.hasParam("loop_hz")) {
        node_.setParam ("loop_hz", 30);       
    }
    if (!node_.hasParam("encoding")) {
        node_.setParam("encoding",  std::string("bgr8"));
    }
    if (!node_.hasParam("filename_pattern")) {
        node_.setParam("filename_pattern",std::string("%s/front%08i.%s"));
    }
    if (!node_.hasParam("mark_based_filtering")) {
        node_.setParam("mark_based_filtering",true);
    }
    if (!node_.hasParam("throttle_based_filtering")) {
        node_.setParam("throttle_based_filtering",true);
    }
    if (!node_.hasParam("throttle_threshold")) {
        node_.setParam("throttle_threshold",0.01);
    }
    if (!node_.hasParam("metadata_shift")) {
        node_.setParam("metadata_shift",5);
    }
}
void RosInterface::updateParam() {
    node_.getParam("loop_hz", loop_hz);
    node_.getParam("encoding", encoding);
    node_.getParam("filename_pattern", filename_pattern);
    node_.getParam("base_path", base_path);
    node_.getParam("mark_based_filtering", mark_based_filtering);
    node_.getParam("throttle_based_filtering", throttle_based_filtering);
    node_.getParam("throttle_threshold", throttle_threshold);
    node_.getParam("metadata_shift", metadata_shift);
    file_format.parse(filename_pattern);
}

void RosInterface::enableCapture () {
    record_data=true;
}
void RosInterface::disableCapture () {
    record_data=false;

}

int dirExists(const char *path)
{
    struct stat info;

    if(stat( path, &info ) != 0)
        return 0;
    else if(info.st_mode & S_IFDIR)
        return 1;
    else
        return 0;
}

void RosInterface::newDataSet (uint32_t mode) {
    imageCount_ = 0;
    datasetCount_++;
    do {
        switch (mode) {
            case 1:
                dataset_path = (dataset_path_format % base_path % date::format("%F", std::chrono::system_clock::now()) % "user" % datasetCount_).str();
            break;
            case 2:
                dataset_path = (dataset_path_format % base_path % date::format("%F", std::chrono::system_clock::now()) % "pilot" % datasetCount_).str();
            break;
            default:
                return;
            break;
        }
        if (dirExists(dataset_path.c_str())==0) {
             mkdir(dataset_path.c_str(), 0777);
             ROS_INFO("New Dataset %s", dataset_path.c_str());
            return;
        }
        datasetCount_++;
    } while (true);
}


void RosInterface::initSub () {

#ifdef SYNCH_TOPICS

    steering_sub.subscribe(node_,"/steering_ctrl/full", 1);
    throttling_sub.subscribe(node_,"/throttle_ctrl/full", 1);
    image_sub.subscribe(node_,"/front_video_resize/image", 1);
    info_sub.subscribe(node_,"/front_video_resize/camera_info", 1);
    //message_filters::Subscriber<robocars_msgs::robocars_actuator_output> steering_sub (node_, "/steering_ctrl/full", 10);
    //message_filters::Subscriber<robocars_msgs::robocars_actuator_output> throttling_sub(node_,"/throttle_ctrl/full",10);
//    message_filters::Subscriber<robocars_msgs::robocars_tof> sensors_tof1_sub(node_,"/sensors/tof1",1);
//    message_filters::Subscriber<robocars_msgs::robocars_tof> sensors_tof2_sub(node_,"/sensors/tof2",1);
//    message_filters::Subscriber<sensor_msgs::Image> image_sub(node_, "/front_video_resize/image", 1);
//    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(node_, "/front_video_resize/camera_info", 1);

#ifdef SIMPLE_SYNC
    message_filters::TimeSynchronizer<   robocars_msgs::robocars_actuator_output, 
                        robocars_msgs::robocars_actuator_output, 
                        robocars_msgs::robocars_tof, 
                        robocars_msgs::robocars_tof, 
                        sensor_msgs::Image, 
                        sensor_msgs::CameraInfo> sync(throttling_sub, steering_sub, sensors_tof1_sub, sensors_tof2_sub, image_sub, info_sub, 10);
#else
    //sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), throttling_sub, steering_sub/*, image_sub, info_sub*/);

#endif
    sync_.reset(new Sync(MySyncPolicy(10), throttling_sub, steering_sub, image_sub, info_sub));
    sync_->registerCallback(boost::bind(&RosInterface::callback,this, _1, _2, _3, _4));
#else
    steering_sub = node_.subscribe<std_msgs::Float32>("/steering_ctrl/norm", 1, &RosInterface::steering_msg_cb, this);
    throttling_sub = node_.subscribe<std_msgs::Float32>("/throttle_ctrl/norm", 1, &RosInterface::throttling_msg_cb, this);
    braking_sub = node_.subscribe<std_msgs::Float32>("/braking_ctrl/norm", 1, &RosInterface::braking_msg_cb, this);
    mark_sub = node_.subscribe<robocars_msgs::robocars_mark>("/annotation/mark", 1, &RosInterface::mark_msg_cb, this);
    //sub_image_and_camera = it->subscribeCamera("/front_video_resize/image", 1, &RosInterface::callbackWithCameraInfo, this);
    sub_image = it->subscribe("/front_video_resize/image", 1, &RosInterface::callbackNoCameraInfo, this);
    //tof1_sub = node_.subscribe<robocars_msgs::robocars_tof>("/sensors/tof1", 1, &RosInterface::tof1_msg_cb, this);
    //tof2_sub = node_.subscribe<robocars_msgs::robocars_tof>("/sensors/tof2", 1, &RosInterface::tof2_msg_cb, this);
    //telem_sub = node_.subscribe<robocars_msgs::robocars_telemetry>("/telemetry",1,&RosInterface::telem_msg_cb, this);
    sensors_sub = node_.subscribe<std_msgs::Int16MultiArray>("/sensors",1,&RosInterface::sensors_msg_cb, this);
#endif
    state_sub = node_.subscribe<robocars_msgs::robocars_brain_state>("/robocars_brain_state", 2, &RosInterface::state_msg_cb, this);

}

void RosInterface::initPub() {
    stats_pub = node_.advertise<robocars_data_capture::robocars_datacapture_stats>("/datacapture/stats", 1);
}


static _Float32 lastSteeringValue=0;
static _Float32 lastThrottlingValue=0;
static _Float32 lastBrakingValue=0;
static _Float32 lastSpeedValue=0;
static _Float32 lastCTEValue=0;
static uint32_t lastTof1Value=0;
static uint32_t lastTof2Value=0;
static uint32_t lastMarkValue=0;
static uint32_t lastLinMarkValue=0;

bool RosInterface::saveImage(const sensor_msgs::ImageConstPtr& image_msg, std::string &jpgFilename) {
    cv::Mat image;
    try
    {
      image = cv_bridge::toCvShare(image_msg, encoding)->image;
    } catch(cv_bridge::Exception)
    {
      ROS_ERROR("Unable to convert %s image to %s", image_msg->encoding.c_str(), encoding.c_str());
      return false;
    }

    if (!image.empty()) {
      try {
        jpgFilename = (file_format % dataset_path % imageCount_ % "jpg").str();
      } catch (...) { file_format.clear(); }

      cv::imwrite(jpgFilename, image);
      if ((imageCount_%100)==0) {
        ROS_INFO("Data Capture: %05ld images saved", imageCount_);
      }
    
    } else {
        ROS_WARN("Couldn't save image, no data!");
        return false;
    }
    return true;
}

bool RosInterface::saveData(const sensor_msgs::ImageConstPtr& image_msg, std::string &jpgFilename) {
   json::JSON obj;
   std::ofstream jsonFile;
   std::string jsonFilename;

   jsonFilename = jpgFilename;
   jsonFilename.replace(jpgFilename.rfind("."), jpgFilename.length(), ".json");
   jsonFile.open(jsonFilename);
   obj["cam/image_array"] = basename(jpgFilename.c_str());
   obj["cam/seq"] = (uint32_t) (image_msg->header.seq);
   obj["ms"] = (uint64_t) (image_msg->header.stamp.toNSec()/1e3);
   obj["angle"] = lastSteeringValue;
   obj["throttle"] = lastThrottlingValue;
   obj["mode"] = drivingMode2str[drivingMode];
   obj["tof1"] = lastTof1Value;
   obj["tof2"] = lastTof2Value;
   obj["mark"] = lastMarkValue;
   obj["linmark"] = lastLinMarkValue;
   obj["brake"] = lastBrakingValue;
   obj["telem/speed"] = lastSpeedValue;
   obj["telem/cte"] = lastCTEValue;
   jsonFile << obj << std::endl;
   jsonFile.close();
 
   return true;
}

#ifdef SYNCH_TOPICS
void RosInterface::callback( const robocars_msgs::robocars_actuator_output::ConstPtr& steering,
                        const robocars_msgs::robocars_actuator_output::ConstPtr& throttling,
                        const sensor_msgs::ImageConstPtr& image, 
                        const sensor_msgs::CameraInfoConstPtr& cam_info) {
    std::string jpgFilename;

    ROS_INFO("Topics Synch]: %05ld", imageCount_);

    lastSteeringValue = steering->norm;
    lastThrottlingValue = throttling->norm;
    if (record_data && ((drivingMode == 1 ) || (drivingMode == 2 ))) {
        if ((drivingMode==1) && (throttle_based_filtering==true && lastThrottlingValue<throttle_threshold)) {
            return;
        }
        if (drivingMode == 1 && mark_based_filtering == true && (lastMarkValue < robocars_msgs::robocars_mark::SWITCH_MARK_2)) {
            return;
        }

        if (!saveImage(image, jpgFilename))
        return;

        // save the metadata
        saveData (image, jpgFilename);
        imageCount_++;
    }
    
}

#else
void RosInterface::callbackNoCameraInfo(const sensor_msgs::ImageConstPtr& image_msg) {
   std::string jpgFilename;

    if (record_data && ((drivingMode == 1 ) || (drivingMode == 2 ))) {
        if ((drivingMode==1) && (throttle_based_filtering==true && lastThrottlingValue<throttle_threshold)) {
            return;
        }
        if (drivingMode == 1 && mark_based_filtering == true && (lastMarkValue < robocars_msgs::robocars_mark::SWITCH_MARK_2)) {
            return;
        }
        if (!saveImage(image_msg, jpgFilename))
            return;

            // save the metadata
        saveData (image_msg, jpgFilename);
        imageCount_++;
    }
}

void RosInterface::callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info) {

    callbackNoCameraInfo(image_msg);
}

void RosInterface::steering_msg_cb(const std_msgs::Float32::ConstPtr& msg){
    lastSteeringValue = msg->data;
}


void RosInterface::throttling_msg_cb(const std_msgs::Float32::ConstPtr& msg){
    lastThrottlingValue = msg->data;
}

void RosInterface::braking_msg_cb(const std_msgs::Float32::ConstPtr& msg){
    lastBrakingValue = msg->data;
}

void RosInterface::telem_msg_cb(const robocars_msgs::robocars_telemetry::ConstPtr& msg){
    lastSpeedValue = msg->speed;
    lastCTEValue = msg->cte;
}

void RosInterface::sensors_msg_cb(const std_msgs::Int16MultiArray::ConstPtr& msg){
    if (msg->data[1]>0) {
        lastSpeedValue = 255.0-(_Float32) mapRange (0,3000,0,255,msg->data[1]);
    } else {
        lastSpeedValue =  0.0;
    }
}

void RosInterface::tof1_msg_cb(const robocars_msgs::robocars_tof::ConstPtr& msg){
    lastTof1Value = msg->distance;
}

void RosInterface::tof2_msg_cb(const robocars_msgs::robocars_tof::ConstPtr& msg){
    lastTof2Value = msg->distance;
}

void RosInterface::mark_msg_cb(const robocars_msgs::robocars_mark::ConstPtr& msg){
    lastMarkValue = msg->mark;
    lastLinMarkValue = msg->linear;
}

#endif

void RosInterface::state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg) {    
    static u_int32_t last_state = -1;
    if (msg->state != last_state) {
        switch (msg->state) {
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE:
                send_event(IdleStatusEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING:
                send_event(ManualDrivingEvent());        
            break;
            case robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING:
                send_event(AutonomousDrivingEvent());        
            break;
        }
        last_state=msg->state;
    }    
}

void RosInterface::reportStats(void) {

    robocars_data_capture::robocars_datacapture_stats statsMsg;
    statsMsg.header.stamp = ros::Time::now();
    statsMsg.header.seq=1;
    statsMsg.header.frame_id = "stats";
    statsMsg.recordedImages = imageCount_;
    statsMsg.currentTub = datasetCount_;
    stats_pub.publish(statsMsg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocars_data_capture");
    
    /*Format : <path>/<date>-<mode>-<index>*/
    dataset_path_format.parse("%s/%s-%s-%02d/");
    ri = new RosInterface;

    ri->initPub();
    fsm_list::start();
    ri->initSub();

    ROS_INFO("Data Capture: Starting");

    // wait for FCU connection
    ros::Rate rate(loop_hz);
    while(ros::ok()){
        ros::spinOnce();
        send_event (TickEvent());
        rate.sleep();
    }
}

