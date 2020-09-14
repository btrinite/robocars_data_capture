/**
 * @file robocars_data_capture.hpp
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
 */

#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>


//#define SYNCH_TOPICS
//#define SIMPLE_SYNC

struct BaseEvent : tinyfsm::Event
{
    public:
        BaseEvent(const char * evtName) : _evtName(evtName) {};
        const char * getEvtName() const { return _evtName; };
    private:
        const char *  _evtName;
};

struct TickEvent                    : BaseEvent { public: TickEvent() : BaseEvent("TickEvent") {}; };
struct IdleStatusEvent              : BaseEvent { public: IdleStatusEvent() : BaseEvent("IdleStatusEvent") {}; };
struct ManualDrivingEvent           : BaseEvent { public: ManualDrivingEvent() : BaseEvent("ManualDrivingEvent") {}; };
struct AutonomousDrivingEvent       : BaseEvent { public: AutonomousDrivingEvent() : BaseEvent("AutonomousDrivingEvent") {}; };
struct RadioChannelEvent            : BaseEvent { public: 
    RadioChannelEvent(const uint32_t value) : radio_channel_value(value), BaseEvent("RadioChannelEvent") {};
    uint32_t radio_channel_value; 
    };

class RobocarsStateMachine
: public tinyfsm::Fsm<RobocarsStateMachine>
{
    public:
        RobocarsStateMachine(const char * stateName) : _stateName(stateName), tinyfsm::Fsm<RobocarsStateMachine>::Fsm() { 
            ROS_INFO("Data capture StateMachine: State created: %s", _stateName);      
        };
        const char * getStateName() const { return _stateName; };

    public:
        /* default reaction for unhandled events */
        void react(BaseEvent const & ev) { 
            ROS_INFO("state %s: unexpected event %s reveived", getStateName(), ev.getEvtName());      
        };

        virtual void react(TickEvent                      const & e) { /*logEvent(e);*/ };
        virtual void react(IdleStatusEvent                const & e) { logEvent(e); };
        virtual void react(ManualDrivingEvent             const & e) { logEvent(e); };
        virtual void react(AutonomousDrivingEvent         const & e) { logEvent(e); };
        virtual void react(RadioChannelEvent              const & e) {  };

        virtual void entry(void) { 
            ROS_INFO("State %s: entering", getStateName()); 
        };  
        void         exit(void)  { };  /* no exit actions */

    private:
        const char *  _stateName ="NoName";
        void logEvent(BaseEvent const & e) {
            ROS_INFO("State %s: event %s", getStateName(), e.getEvtName());
        }
};

typedef tinyfsm::FsmList<RobocarsStateMachine> fsm_list;

template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}



class RosInterface
{
    public :
        RosInterface() : node_("~") {
            initParam();
            updateParam();
            #ifndef SYNCH_TOPICS
            it = new image_transport::ImageTransport(node_);
            #endif
            imageCount_= 0;
            datasetCount_ = 0;
            record_data = false;
        };


        void initParam();
        void updateParam();
        void initSub();

        void enableCapture ();
        void disableCapture ();
        void newDataSet (uint32_t mode);

    private:

        void state_msg_cb(const robocars_msgs::robocars_brain_state::ConstPtr& msg);

        bool saveImage(const sensor_msgs::ImageConstPtr& image_msg, std::string &jpgFilename);
        bool saveData(const sensor_msgs::ImageConstPtr& image_msg, std::string &jpgFilename);

        bool record_data;
        size_t imageCount_;
        size_t datasetCount_;

        ros::NodeHandle node_;
        ros::Subscriber state_sub;

#ifdef SYNCH_TOPICS
        void callback(  const robocars_msgs::robocars_actuator_output::ConstPtr& steering,
                        const robocars_msgs::robocars_actuator_output::ConstPtr& throttling,
                        const robocars_msgs::robocars_tof::ConstPtr& tof1,
                        const robocars_msgs::robocars_tof::ConstPtr& tof2,
                        const sensor_msgs::ImageConstPtr& image, 
                        const sensor_msgs::CameraInfoConstPtr& cam_info);
#else
        void callbackNoCameraInfo(const sensor_msgs::ImageConstPtr& image_msg);
        void callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info);
        void steering_msg_cb(const robocars_msgs::robocars_actuator_output::ConstPtr& msg);
        void throttling_msg_cb(const robocars_msgs::robocars_actuator_output::ConstPtr& msg);
        void braking_msg_cb(const robocars_msgs::robocars_actuator_output::ConstPtr& msg);
        void mark_msg_cb(const robocars_msgs::robocars_mark::ConstPtr& msg);
        void tof1_msg_cb(const robocars_msgs::robocars_tof::ConstPtr& msg);
        void tof2_msg_cb(const robocars_msgs::robocars_tof::ConstPtr& msg);

        ros::Subscriber throttling_sub;
        ros::Subscriber steering_sub;
        ros::Subscriber braking_sub;
        ros::Subscriber tof1_sub;
        ros::Subscriber tof2_sub;
        ros::Subscriber mark_sub;
        image_transport::ImageTransport * it;
        image_transport::CameraSubscriber sub_image_and_camera;
        image_transport::Subscriber sub_image;
#endif

};

