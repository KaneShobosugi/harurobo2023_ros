// main branchにマージしないこと。ビルド通すための設定に費やす時間がないので。

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

#include "../include/collector_node_variable.hpp"

namespace collector_node
{

    class Collector : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber joySub_;
        ros::Publisher canPub_;
        ros::Publisher toSolenoidValveBoardPub_;

        // enum class ArmMode
        // {
        //     inside,
        //     standstill,
        //     outside,
        // };

        // onesideEquipment rightEquipment;
        // onesideEquipment leftEquipment;

        // ArmMode previousArmMode1{ArmMode::standstill};
        // ArmMode previousArmMode2{ArmMode::standstill};

        bool ledState{false};
        static std::array<bool, 11> previousButtonInput; // invalid use of non-static data member
        std::array<bool, 11> currentButtonInput;

    public:
        void onInit()
        {
            // rightEquipment.collectionOBJ.solenoidPortNo = 0;

            joySub_ = nh_.subscribe("joy", 1, &Collector::joyCallback, this);
            canPub_ = nh_.advertise<can_plugins::Frame>("can_tx", 1);
            toSolenoidValveBoardPub_ = nh_.advertise<harurobo2023_ros::toSolenoidValveBoardDriverTopic>("solenoidValveBoard", 10);
            NODELET_INFO("'collector_node' has started.");
        }

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr &_joy)
        {
            can_plugins::Frame frame;
            harurobo2023_ros::toSolenoidValveBoardDriverTopic toSolenoidValveBoardDriverTopicFrame;

            for (int i = 0; i <= 11; i++)
            {
                currentButtonInput[i] = (_joy->buttons[i]);
            }

            // LED function start
            if (_joy->buttons[8])
            {
                ledState = !ledState;

                toSolenoidValveBoardDriverTopicFrame.portNo = 4;
                toSolenoidValveBoardDriverTopicFrame.isOn = ledState;
                toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);
            }
            // LED function end

            // main function start
            // 同時押しセット1 right side
            if (_joy->buttons[5])
            {
                can_plugins::Frame frame;
                harurobo2023_ros::toSolenoidValveBoardDriverTopic toSolenoidValveBoardDriverTopicFrame;

                const uint16_t collection_solenoidPortNo{solenoidValveBoard::no0};
                const uint16_t collection_steppingMotorNo{0};
                const float collection_steppingMotorPosDisplacement{collector_node_variable::collection_steppingMotorPosDisplacement};
                const float collection_armRotationTime{collector_node_variable::collection_armRotationTime};
                const float valveSwitchTime{2};
                const uint16_t ejection_solenoidPortNo{0};
                const uint16_t ejection_shirasuID{0x000};
                const float ejectionTime{collector_node_variable::ejectionTime};
                const float ejectionleverRotationTime{collector_node_variable::ejectionleverRotationTime};

                uint16_t steppingMotorID{collector_node_variable::baseBoardForSteppingMotorID + 4 * collection_steppingMotorNo};


                ros::Time time_start;

                if (_joy->buttons[0] && !previousButtonInput[0])
                {
                    frame = get_frame(steppingMotorID + 0, baseBoardForSteppingMotor_setting::BIDplus0_Cmd::position_mode);
                    canPub_.publish(frame);

                    frame = get_frame(steppingMotorID + 1, collection_steppingMotorPosDisplacement);
                    canPub_.publish(frame);

                    time_start = ros::Time::now();
                    // multiple command to can
                    while ((ros::Time::now() - time_start).sec < collection_armRotationTime)
                    {
                        volatile int dummy = 0;
                    }

                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no0;
                    toSolenoidValveBoardDriverTopicFrame.isOn = true; // open
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);

                    time_start = ros::Time::now();
                    // multiple command to can
                    while ((ros::Time::now() - time_start).sec < valveSwitchTime)
                    {
                        volatile int dummy = 0;
                    }

                    frame = get_frame(steppingMotorID + 0, baseBoardForSteppingMotor_setting::BIDplus0_Cmd::position_mode);
                    canPub_.publish(frame);

                    frame = get_frame(steppingMotorID + 1, -collection_steppingMotorPosDisplacement);
                    canPub_.publish(frame);

                    time_start = ros::Time::now();
                    // multiple command to can
                    while ((ros::Time::now() - time_start).sec < collection_armRotationTime)
                    {
                        volatile int dummy = 0;
                    }
                }
                else if (_joy->buttons[1] && !previousButtonInput[1])
                {

                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no2;
                    toSolenoidValveBoardDriverTopicFrame.isOn = true; // open
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);

                    time_start = ros::Time::now();
                    // multiple command to can
                    while ((ros::Time::now() - time_start).sec < ejectionTime)
                    {
                        volatile int dummy = 0;
                    }

                    frame = get_frame(ejection_shirasuID, shirasu_setting::BIDplus0_Cmd::homing_mode);
                    canPub_.publish(frame);

                    time_start = ros::Time::now();
                    // multiple command to can
                    while ((ros::Time::now() - time_start).sec < ejectionleverRotationTime)
                    {
                        volatile int dummy = 0;
                    }

                    toSolenoidValveBoardDriverTopicFrame.portNo = solenoidValveBoard::no2;
                    toSolenoidValveBoardDriverTopicFrame.isOn = false; // close
                    toSolenoidValveBoardPub_.publish(toSolenoidValveBoardDriverTopicFrame);
                }
                else if (_joy->axes[7] < 0) // bend the arm to the inside direction
                {
                    frame = get_frame(steppingMotorID + 0, baseBoardForSteppingMotor_setting::BIDplus0_Cmd::velocity_mode);
                    canPub_.publish(frame);

                    frame = get_frame(steppingMotorID + 1,collector_node_variable::collection_steppingVel );
                    canPub_.publish(frame);
                }
                else if (_joy->axes[7] > 0) // bend the arm to the outside direction
                {
                }
                else if (_joy->axes[7] == 0)
                {
                }

                // 同時押しセット2 left side
                else if (_joy->buttons[4])
                {
                }

                // main function end

                previousButtonInput = currentButtonInput;
            };
        };
    };
} // namespace collector_node
PLUGINLIB_EXPORT_CLASS(collector_node::Collector, nodelet::Nodelet)
