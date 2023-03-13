#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <can_plugins/Frame.h>
#include <../../can_plugins/include/can_utils.hpp>

#include <solenoidValveBoard.hpp>
#include <harurobo2023_ros/toSolenoidValveBoardDriverTopic.h>

#define PORT_NUMBER 5

namespace solenoidValveBoardDriver_node
{

    class SolenoidValveBoardDriver : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber solenoidValveSub_;
        ros::Publisher canPub_;

        // uint8_t previousSentDirection{}; // 0b00000000
        std::array<bool, PORT_NUMBER> previousValveState{};
        std::array<bool, PORT_NUMBER> currentValveState{};

        const uint16_t solenoidValveBoardID{0x100}; // EDIT

    public:
        void
        onInit()
        {
            nh_ = getNodeHandle();
            solenoidValveSub_ = nh_.subscribe("solenoidValveBoard", 1, &SolenoidValveBoardDriver::solenoidValveCallback, this);
            canPub_ = nh_.advertise<can_plugins::Frame>("can_tx", 100);
            NODELET_INFO("'solenoidValveBoardDriver_node' has started.");
        }

    private:
        void solenoidValveCallback(const harurobo2023_ros::toSolenoidValveBoardDriverTopic::ConstPtr &_solenoid)
        {
            // code with a construe
            /*
            can_plugins::Frame frame;
            uint8_t currentToSendDirection{};

            if (_solenoid->isOn == true)
            {
                currentToSendDirection = (previousSentDirection | 1 << (_solenoid->portNo));
            }
            else
            {
                currentToSendDirection = (previousSentDirection & !(1 << _solenoid->portNo));
            }

            if (currentToSendDirection != previousSentDirection)
            {
                frame = get_frame(solenoidValveBoardID, currentToSendDirection);
                canPub_.publish(frame);

                previousSentDirection = currentToSendDirection;

                ROS_INFO("debug: solenoidValveCallback 01");
            }
            */

            // new code
            can_plugins::Frame frame;
            if (_solenoid->isOn == true)
            {
                currentValveState[_solenoid->portNo] = true;
            }
            else
            {
                currentValveState[_solenoid->portNo] = false;
            }

            // if (currentValveState != previousValveState)
            {
                previousValveState = currentValveState;

                frame = get_frame(solenoidValveBoardID + 0, (uint8_t)1); // turn on
                canPub_.publish(frame);

                uint8_t command{}; // to solenoid valve board
                for (int i = 0; i < PORT_NUMBER; i++)
                {
                    command += currentValveState[i] << i;
                }
                frame = get_frame(solenoidValveBoardID + 1, command);
                canPub_.publish(frame);
            }
        };
    };
} // namespace solenoidValveBoardDriver_node
PLUGINLIB_EXPORT_CLASS(solenoidValveBoardDriver_node::SolenoidValveBoardDriver, nodelet::Nodelet)
