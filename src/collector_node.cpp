#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <can_plugins/Frame.h>
#include <../../can_plugins/include/can_utils.hpp>

#include <shirasu_setting.hpp>

namespace collector_node
{

    class Collector : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber joySub_;
        ros::Publisher canPub_;

        uint16_t solenoidValveBoardID{0x100};        // EDIT
        uint16_t baseBoardForSteppingMotorID{0x010}; // EDIT

    public:
        void
        onInit()
        {
            nh_ = getNodeHandle();
            joySub_ = nh_.subscribe("joy", 1, &Collector::joyCallback, this);
            canPub_ = nh_.advertise<can_plugins::Frame>("can_tx", 1);
            NODELET_INFO("'collector_node' has started.");
        }

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr &_joy)
        {
            // jikan de joutai senni
            if (_joy->axes[7] > 0) // bend the arm to the inside direction.
            {
                can_plugins::Frame frame;
                float radian{+1 * M_PI};

                frame = get_frame(baseBoardForSteppingMotorID + 4 * 0 + 0, 4); // position mode
                canPub_.publish(frame);

                frame = get_frame(baseBoardForSteppingMotorID + 4 * 0 + 1, radian); // target
                canPub_.publish(frame);
            }
            else if (_joy->axes[7] < 0) // bend the arm to the outside direction.
            {
                can_plugins::Frame frame;
                float radian{-1 * M_PI};

                frame = get_frame(baseBoardForSteppingMotorID + 4 * 0 + 0, uint8_t(4)); // position mode
                canPub_.publish(frame);

                frame = get_frame(baseBoardForSteppingMotorID + 4 * 0 + 1, radian); // target
                canPub_.publish(frame);
            }

            //
            if (_joy->axes[6] > 0) // hold
            {
                can_plugins::Frame frame;
                frame = get_frame(solenoidValveBoardID, uint8_t(1 << 0)); // port no 0
                canPub_.publish(frame);
            }
            else if (_joy->axes[6] < 0) // unleash
            {
                can_plugins::Frame frame;
                frame = get_frame(solenoidValveBoardID, uint8_t(0 << 0));
                canPub_.publish(frame);
            }
        };
    };
} // namespace collector_node
PLUGINLIB_EXPORT_CLASS(collector_node::Collector, nodelet::Nodelet)
