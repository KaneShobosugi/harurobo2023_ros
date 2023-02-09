// This program is designed for an undercarriage with 4 wheels; however, it will be able to support one with 3 wheels with the least amount of modifications.

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <numeric>

#include <can_plugins/Frame.h>
#include <../../can_plugins/include/can_utils.hpp>

#include <shirasu_setting.hpp>

namespace undercarriage_node
{
    class Wheel
    {
    public:
        const uint16_t id;
        const std::array<double, 2> wheelPosition;
        const double k1{0.1}; // EDIT
        const double k2{0.1}; // EDIT
        const double k3;

        std::array<double, 2> wheelDirection;

    public:
        Wheel(const uint16_t _id, const std::array<double, 2> _wheelPosition, const double _k3)
            : id{_id}, wheelPosition{_wheelPosition}, k3{_k3}
        {
            wheelDirection = {(-1) * wheelPosition[1], wheelPosition[0]};
        }

        // i maybe will delete this
        double toWheelAngularVelocity(const std::array<double, 2> &_linearVelosityViaJoy, const double &_angularVelosityViaJoy)
        {
            double WheelAngularVelocity;
            WheelAngularVelocity = k3 * (k1 * (std::inner_product(_linearVelosityViaJoy.begin(), _linearVelosityViaJoy.end(), wheelDirection.begin(), 0.0)) + k2 * (_angularVelosityViaJoy));
            return WheelAngularVelocity;
        }
    };

    class Undercarriage : public nodelet::Nodelet
    {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber joySub_;
        ros::Publisher canPub_;

        std::array<Wheel, 4> wheelArray = {
            Wheel(0x130, {+1.0, +1.0}, +1.0),
            Wheel(0x120, {-1.0, +1.0}, +1.0),
            Wheel(0x110, {-1.0, -1.0}, +1.0),
            Wheel(0x100, {+1.0, -1.0}, +1.0),
        }; // EDIT

    public:
        void
        onInit()
        {
            nh_ = getNodeHandle();
            joySub_ = nh_.subscribe("joy", 1, &Undercarriage::joyCallback, this); // template hikisuu?
            canPub_ = nh_.advertise<can_plugins::Frame>("can_tx", 1);
            NODELET_INFO("'undercarriage_node' has started.");
        }

    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr &_joy)
        {
            // Right Stick Button
            if (_joy->buttons[10] == 1)
            {
                for (const auto &_wheel : wheelArray)
                {
                    can_plugins::Frame frame;
                    frame = get_frame(_wheel.id + 0, shirasu_setting::BIDplus0_Cmd::velocity_mode);
                    canPub_.publish(frame);
                }
            }
            // Left Stick Button
            if (_joy->buttons[9] == 1)
            {
                for (const auto &_wheel : wheelArray)
                {
                    can_plugins::Frame frame;
                    frame = get_frame(_wheel.id + 0, shirasu_setting::BIDplus0_Cmd::disable_mode);
                    canPub_.publish(frame);
                }
            }

            // on the main purpuse
            std::array<double, 2> linearVelosityViaJoy; //(x,y)
            double angularVelosityViaJoy;

            linearVelosityViaJoy[0] = (+1) * (_joy->axes[0]);
            linearVelosityViaJoy[1] = (-1) * (_joy->axes[1]);
            angularVelosityViaJoy = (-1) * (_joy->axes[3]);

            for (auto &_wheel : wheelArray)
            {
                can_plugins::Frame frame;
                frame = get_frame(_wheel.id + 1, (_wheel.toWheelAngularVelocity(linearVelosityViaJoy, angularVelosityViaJoy)));
                canPub_.publish(frame);
            }
        };
    };
} // namespace undercarriage_node
PLUGINLIB_EXPORT_CLASS(undercarriage_node::Undercarriage, nodelet::Nodelet)
