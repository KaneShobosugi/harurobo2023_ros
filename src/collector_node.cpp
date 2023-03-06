#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <can_plugins/Frame.h>
#include <../../can_plugins/include/can_utils.hpp>

#include <shirasu_setting.hpp>
#include <baseBoardForSteppingMotor_setting.hpp>
#include <solenoidValveBoard.hpp>
#include <harurobo2023_ros/toSolenoidValveBoardDriverTopic.h>

namespace collector_node
{

    class Collector : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber joySub_;
        ros::Publisher canPub_;
        ros::Publisher toSolenoidValveBoardPub_;

        // uint16_t solenoidValveBoardID{0x100};        // EDIT
        uint16_t baseBoardForSteppingMotorID{0x010}; // EDIT

        enum class ArmMode
        {
            inside,
            standstill,
            outside,
        };

        ArmMode previousArmMode1{ArmMode::standstill};
        ArmMode previousArmMode2{ArmMode::standstill};

    public:
        void
        onInit()
        {
            nh_ = getNodeHandle();
            joySub_ = nh_.subscribe("joy", 1, &Collector::joyCallback, this);
            canPub_ = nh_.advertise<can_plugins::Frame>("can_tx", 1);
            toSolenoidValveBoardPub_ = nh_.advertise<harurobo2023_ros::toSolenoidValveBoardDriverTopic>("solenoidValveBoard", 10);
            NODELET_INFO("'collector_node' has started.");
        }

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr &_joy)
        {
            const float velocityValue{1.0}; // EDIT

            if (_joy->buttons[5])
            {

                int steppingMotorNo{0}; // EDIT 0~3

                if (_joy->axes[7] < 0) // bend the arm to the inside direction.
                {
                    can_plugins::Frame frame;

                    if (previousArmMode1 != ArmMode::inside)
                    {
                        frame = get_frame(baseBoardForSteppingMotorID + 4 * steppingMotorNo + 0, baseBoardForSteppingMotor_setting::BIDplus0_Cmd::velocity_mode);
                        canPub_.publish(frame);

                        frame = get_frame(baseBoardForSteppingMotorID + 4 * steppingMotorNo + 1, velocityValue); // target
                        canPub_.publish(frame);

                        previousArmMode1 = ArmMode::inside;
                    }
                }
                else if (_joy->axes[7] > 0) // bend the arm to the outside direction.
                {
                    can_plugins::Frame frame;

                    if (previousArmMode1 != ArmMode::outside)
                    {
                        frame = get_frame(baseBoardForSteppingMotorID + 4 * steppingMotorNo + 0, baseBoardForSteppingMotor_setting::BIDplus0_Cmd::velocity_mode); //
                        canPub_.publish(frame);

                        frame = get_frame(baseBoardForSteppingMotorID + 4 * steppingMotorNo + 1, (-1) * velocityValue); // target
                        canPub_.publish(frame);

                        previousArmMode1 = ArmMode::outside;

                        ROS_INFO("debug: collector_node stepping");
                    }
                }
                else if (_joy->axes[7] == 0)
                {
                    can_plugins::Frame frame;

                    if (previousArmMode1 != ArmMode::standstill)
                    {
                        frame = get_frame(baseBoardForSteppingMotorID + 4 * steppingMotorNo + 0, baseBoardForSteppingMotor_setting::BIDplus0_Cmd::disable_mode); //
                        canPub_.publish(frame);

                        previousArmMode1 = ArmMode::standstill;
                    }
                }

                //
                harurobo2023_ros::toSolenoidValveBoardDriverTopic toSolenoidValveBoardDriverTopicFrame;
                if (_joy->axes[6] < 0) // hold
                {
                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no0;
                    toSolenoidValveBoardDriverTopicFrame.isOn = true;
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);
                }
                else if (_joy->axes[6] > 0) // unleash
                {
                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no0;
                    toSolenoidValveBoardDriverTopicFrame.isOn = false;
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);
                }
            }
            else if (_joy->buttons[4]) // 同時押し
            {
            }
        };
    };
} // namespace collector_node
PLUGINLIB_EXPORT_CLASS(collector_node::Collector, nodelet::Nodelet)
