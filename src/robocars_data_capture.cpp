/**
 * @file offb_raw_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 source $src_path/Tools/setup_gazebo.bash ${src_path} ${build_path}

 gzserver --verbose ${src_path}/Tools/sitl_gazebo/worlds/${model}.world &
 */
#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>

#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <date.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_actuator_ctrl_mode.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_brain_state.h>

#include <robocars_data_capture.hpp>

RosInterface * ri;

static int loop_hz;
static std::string encoding;
static std::string filename_pattern;
static std::string base_path;
static std::string dataset_path;
static boost::format file_format;
static boost::format dataset_path_format;


class onRunningMode;
class onIdle;
class onManualDriving;
class onAutonomousDriving;
class onQualibtrateMode;

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
            ri->newDataSet();
            ri->enableCapture();
        };

        void react (AutonomousDrivingEvent const & e) override {
            onRunningMode::react(e);
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
        };  

        virtual void react(IdleStatusEvent                 const & e) override { 
            onRunningMode::react(e);
            transit<onIdle>();
        };

        virtual void react(ManualDrivingEvent                     const & e) override { 
            onRunningMode::react(e);
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
        node_.setParam("filename_pattern",std::string("/front%04i.%s"));
    }
}
void RosInterface::updateParam() {
    node_.getParam("loop_hz", loop_hz);
    node_.getParam("encoding", encoding);
    node_.getParam("filename_pattern", filename_pattern);
    node_.getParam("base_path", base_path);
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

void RosInterface::newDataSet () {
    imageCount_ = 0;
    datasetCount_++;
    do {
        dataset_path = (dataset_path_format % base_path % date::format("%F", std::chrono::system_clock::now()) % datasetCount_).str();
        if (dirExists(dataset_path.c_str())==0) {
             mkdir(dataset_path.c_str(), 0777);
            return;
        }
        datasetCount_++;
    } while (true);
}


void RosInterface::initSub () {
    channels_sub = node_.subscribe<robocars_msgs::robocars_radio_channels>("/radio_channels", 1, &RosInterface::channels_msg_cb, this);
    state_sub = node_.subscribe<robocars_msgs::robocars_brain_state>("/robocars_brain_state", 1, &RosInterface::state_msg_cb, this);
    sub_image_and_camera = it->subscribeCamera("/front_video_resize/image", 1, &RosInterface::callbackWithCameraInfo, this);
}

bool RosInterface::saveImage(const sensor_msgs::ImageConstPtr& image_msg, std::string &filename) {
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
        filename = (file_format % dataset_path % imageCount_ % "jpg").str();
      } catch (...) { file_format.clear(); }

      cv::imwrite(filename, image);
      ROS_INFO("Saved image %s", filename.c_str());

    } else {
        ROS_WARN("Couldn't save image, no data!");
        return false;
    }
    return true;
}


void RosInterface::callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info) {

   std::string filename;

    if (record_data) {
        if (!saveImage(image_msg, filename))
        return;

        // save the CameraInfo
        if (info) {
        filename = filename.replace(filename.rfind("."), filename.length(), ".ini");
        }
        
        imageCount_++;
    }
}

void RosInterface::channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg){
    
}

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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robocars_data_capture");
    
    dataset_path_format.parse("%s/%s-run-%02d/");
    ri = new RosInterface;

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

