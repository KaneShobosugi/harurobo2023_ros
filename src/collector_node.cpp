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

        class onesideEquipment
        {
        private:
            can_plugins::Frame frame;
            harurobo2023_ros::toSolenoidValveBoardDriverTopic toSolenoidValveBoardDriverTopicFrame;

            class collection
            {
            public:
                const uint16_t solenoidPortNo;
                const uint16_t steppingMotorNo;

                const float steppingMotorPosDisplacement;
            };
            collection collectionOBJ;

            class ejection
            {
            public:
                const uint16_t solenoidPortNo;
                const uint16_t shirasuID;
            };
            ejection ejectionOBJ;
            int test{};

        public:
            onesideEquipment(){

            };

            void keyCallback(const sensor_msgs::Joy::ConstPtr &_joy)
            {
                uint16_t steppingMotorID{collector_node_variable::baseBoardForSteppingMotorID+4*collection::steppingMotorNo};

                if (_joy->buttons[0] && !previousButtonInput[0])
                {
                    frame = get_frame(steppingMotorID+0,baseBoardForSteppingMotor_setting::BIDplus0_Cmd::position_mode);
                    canPub_.publish
                }
                else if (_joy->buttons[1] && !previousButtonInput[1])
                {
                }
            };
        }; // class onesideEquipment definition
        onesideEquipment rightEquipment;
        onesideEquipment leftEquipment;

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
                rightEquipment.keyCallback(_joy);
            }
            // 同時押しセット2 left side
            else if (_joy->buttons[4])
            {
            }

            // main function end

            previousButtonInput = currentButtonInput;
        };
    };

} // namespace collector_node
PLUGINLIB_EXPORT_CLASS(collector_node::Collector, nodelet::Nodelet)
