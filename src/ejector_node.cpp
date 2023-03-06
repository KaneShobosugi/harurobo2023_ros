#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <can_plugins/Frame.h>
#include <../../can_plugins/include/can_utils.hpp>

#include <shirasu_setting.hpp>

#include <solenoidValveBoard.hpp>
#include <harurobo2023_ros/toSolenoidValveBoardDriverTopic.h>

#define MODE_G

namespace ejector_node
{
    enum class EjectorState
    {
        unlocked,
        locked,
        ready,
    };

    class Ejector : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber joySub_;
        ros::Publisher canPub_;
        ros::Publisher toSolenoidValveBoardPub_;

        uint16_t shirasuID_1{0x004}; // EDIT
        uint16_t shirasuID_2{0x004}; // EDIT

        // uint16_t solenoidValveBoardID{0x008}; // EDIT
        EjectorState currentEjectorState{EjectorState::unlocked};

        // bool debugMode{true}; // EDIT

    public:
        void
        onInit()
        {
            nh_ = getNodeHandle();
            joySub_ = nh_.subscribe("joy", 1, &Ejector::joyCallback, this);
            canPub_ = nh_.advertise<can_plugins::Frame>("can_tx", 10);
            toSolenoidValveBoardPub_ = nh_.advertise<harurobo2023_ros::toSolenoidValveBoardDriverTopic>("solenoidValveBoard", 10);

            NODELET_INFO("'ejector_node' has started.");
        }

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr &_joy)
        {
#ifndef MODE_G
            // ros::Timer timer=nh_.createTimer(ros::Duration(0.1),);

            // STARTボタンが押された場合。
            if (_joy->buttons[7])
            {
                can_plugins::Frame frame;

                ros::Time time_start = ros::Time::now();
                frame = get_frame(shirasuID + 0, shirasu_setting::BIDplus0_Cmd::position_mode);
                canPub_.publish(frame);
                ROS_INFO("en1");

                constexpr double highVelocity{10.0}; // EDIT
                frame = get_frame(shirasuID + 1, highVelocity);
                canPub_.publish(frame);
                ROS_INFO("en2");

                const double highVelocityContinuationTime{1.5}; // EDIT
                while ((ros::Time::now() - time_start).toSec() < highVelocityContinuationTime)
                {
                    volatile int dummy = 0;
                }
                const double lowVelocity{2.0}; // EDIT
                frame = get_frame(shirasuID + 1, lowVelocity);
                canPub_.publish(frame);
                ROS_INFO("en3");

                const double lowVelocityContinuationTime{0.5}; // EDIT
                while ((ros::Time::now() - time_start).toSec() < lowVelocityContinuationTime)
                {
                    volatile int dummy = 0;
                }
                frame = get_frame(shirasuID + 0, shirasu_setting::BIDplus0_Cmd::homing_mode);
                canPub_.publish(frame);
                ROS_INFO("en4");
            }
#else
            can_plugins::Frame frame;
            // const uint8_t portNo2{2};                    // EDIT
            const double waitingTime{5.0};               // EDIT
            const double positionModeDisplacement{10.0}; // EDIT

            harurobo2023_ros::toSolenoidValveBoardDriverTopic toSolenoidValveBoardDriverTopicFrame;

            if (_joy->buttons[5])
            {
                if (_joy->buttons[0])
                {
                    frame = get_frame(shirasuID_1 + 0, shirasu_setting::BIDplus0_Cmd::homing_mode);
                    canPub_.publish(frame);
                }
                else if (_joy->buttons[1])
                {
                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no2;
                    toSolenoidValveBoardDriverTopicFrame.isOn = true;
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);

                    ros::Time time_start = ros::Time::now();
                    while ((ros::Time::now() - time_start).toSec() < waitingTime)
                    {
                        volatile int dummy = 0;
                    }

                    frame = get_frame(shirasuID_1 + 0, shirasu_setting::BIDplus0_Cmd::position_mode);
                    canPub_.publish(frame);

                    frame = get_frame(shirasuID_1 + 1, positionModeDisplacement);
                    canPub_.publish(frame);
                }
                else if (_joy->buttons[2])
                {
                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no2;
                    toSolenoidValveBoardDriverTopicFrame.isOn = false;
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);
                }
            }
            else if(_joy->buttons[4])
            {
                if (_joy->buttons[0])
                {
                    frame = get_frame(shirasuID_2 + 0, shirasu_setting::BIDplus0_Cmd::homing_mode);
                    canPub_.publish(frame);
                }
                else if (_joy->buttons[1])
                {
                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no3;
                    toSolenoidValveBoardDriverTopicFrame.isOn = true;
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);

                    ros::Time time_start = ros::Time::now();
                    while ((ros::Time::now() - time_start).toSec() < waitingTime)
                    {
                        volatile int dummy = 0;
                    }

                    frame = get_frame(shirasuID_2 + 0, shirasu_setting::BIDplus0_Cmd::position_mode);
                    canPub_.publish(frame);

                    frame = get_frame(shirasuID_2 + 1, positionModeDisplacement);
                    canPub_.publish(frame);
                }
                else if (_joy->buttons[2])
                {
                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no3;
                    toSolenoidValveBoardDriverTopicFrame.isOn = false;
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);
                }
            }

#endif
        };
    };
} // namespace ejector_node
PLUGINLIB_EXPORT_CLASS(ejector_node::Ejector, nodelet::Nodelet)
