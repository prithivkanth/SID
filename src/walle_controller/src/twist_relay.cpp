#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"



class TwistRelay : public rclcpp::Node  
{
public:
    TwistRelay() : Node("twist_relay")
    {
        // for keyboard teleop like control

        controller_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/walle_controller/cmd_vel_unstamped",
            10,
            std::bind(&TwistRelay::controller_twist_callback,this, std::placeholders::_1)
        );

        controller_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/walle_controller/cmd_vel",
            10
        );

        // for joy teleop twist relay
        
        joy_sub_ = this -> create_subscription<geometry_msgs::msg::TwistStamped>(
            "/input_joy/cmd_vel_stamped",
            10,
            std::bind(&TwistRelay::joy_twist_callback, this, std::placeholders::_1)
        );

        joy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/input/cmd_vel",
            10
        );

    }


private:

    // for keyboard teleop like control
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr controller_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_pub_;

    // for joy teleop twist relay
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr joy_pub_;

    // callback logic
    

    void controller_twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped twiststamped;
        twiststamped.header.stamp = get_clock()->now();
        twiststamped.twist = *msg;
        controller_pub_->publish(twiststamped);
    }

    void joy_twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        geometry_msgs::msg::Twist twist;
        twist = msg->twist;
        joy_pub_->publish(twist);
    }

};


int main(int argc, char* argv[])
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TwistRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}