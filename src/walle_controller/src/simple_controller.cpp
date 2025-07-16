#include "walle_controller/simple_controller.hpp"
#include <Eigen/Geometry>

using std::placeholders::_1;


SimpleController::SimpleController(const std::string &name) : Node(name)
    , left_wheel_prev_pos_(0.0)
    , right_wheel_prev_pos_(0.0)
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)

{
    declare_parameter("wheel_radius", 0.054);
    declare_parameter("wheel_separation", 0.31);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "the radius is " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "the seperation is " << wheel_separation_);

    prev_time_ = get_clock()->now();

    wheel_cmd_pub_= create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_= create_subscription<geometry_msgs::msg::TwistStamped>("/walle_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1));

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1)); 

    speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_;
    RCLCPP_INFO_STREAM(get_logger(), "the speed conversion is " << speed_conversion_);


}

void SimpleController::velCallback (const geometry_msgs::msg::TwistStamped &msg)
{
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z);

    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed;

    std_msgs::msg::Float64MultiArray wheel_speed_msg;

    wheel_speed_msg.data.push_back(wheel_speed.coeff(1));
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0));

    wheel_cmd_pub_->publish(wheel_speed_msg);

}


void SimpleController::jointCallback(const sensor_msgs::msg::JointState &msg)
{
    double dp_left = msg.position.at(1) - left_wheel_prev_pos_;
    double dp_right = msg.position.at(0) - right_wheel_prev_pos_;

    rclcpp::Time msg_time = msg.header.stamp;
    rclcpp::Duration dt = msg_time - prev_time_;

    left_wheel_prev_pos_= msg.position.at(1);
    right_wheel_prev_pos_= msg.position.at(0);
    prev_time_ = msg.header.stamp;


    double fi_left = dp_left/dt.seconds();
    double fi_right = dp_right/dt.seconds();

    double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left)/2;
    double angular =  (wheel_radius_ * fi_left - wheel_radius_ * fi_right)/wheel_separation_;


     // Calculate the position increment
    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;
    theta_ += d_theta;
    x_ += d_s * cos(theta_);
    y_ += d_s * sin(theta_);


    RCLCPP_INFO_STREAM(get_logger(), "Linear : "<<linear<<" Angular : "<<angular);

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}