#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <can_plugins/Frame.h>
#include <../../can_plugins/include/can_utils.hpp>

#include <shirasu_setting.hpp>

namespace ejector_node
{

    class Ejector : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber joySub_;
        ros::Publisher canPub_;

    public:
        void
        onInit()
        {
            nh_ = getNodeHandle();
            joySub_ = nh_.subscribe("joy", 1, &Ejector::joyCallback, this);
            canPub_ = nh_.advertise<can_plugins::Frame>("can_tx", 1);
            NODELET_INFO("'ejector_node' has started.");
        }

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr &_joy){

        };
    };
} // namespace undercarriage_node
PLUGINLIB_EXPORT_CLASS(ejector_node::Ejector, nodelet::Nodelet)
