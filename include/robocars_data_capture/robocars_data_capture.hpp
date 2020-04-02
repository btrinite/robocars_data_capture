#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>


#define SYNCH_TOPICS

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
        void newDataSet ();

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
        callbackWithCameraInfo(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info);
        steering_msg_cb(const robocars_msgs::robocars_actuator_output::ConstPtr& msg);
        throttling_msg_cb(const robocars_msgs::robocars_actuator_output::ConstPtr& msg);

        ros::Subscriber throttling_sub;
        ros::Subscriber steering_sub;
        image_transport::ImageTransport * it;
        image_transport::CameraSubscriber sub_image_and_camera;
#endif

};

