#include "walle_localization/odometry_motion_model.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/utils.h>

using std::placeholders::_1;


OdometryMotionModel::OdometryMotionModel(const std::string& name)
                    : Node(name),
                    alpha1_(0.0),
                    alpha2_(0.0),
                    alpha3_(0.0),
                    alpha4_(0.0),
                    no_samples_(300),
                    last_odom_x_(0.0),
                    last_odom_y_(0.0),
                    last_odom_theta_(0.0),
                    is_first_odom_(true)

{   
    declare_parameter("alpha1", 0.1);
    declare_parameter("alpha2", 0.1);
    declare_parameter("alpha3", 0.1);
    declare_parameter("alpha4", 0.1);
    declare_parameter("nr_samples", 300);

    alpha1_ = get_parameter("alpha1").as_double();
    alpha2_ = get_parameter("alpha2").as_double();
    alpha3_ = get_parameter("alpha3").as_double();
    alpha4_ = get_parameter("alpha4").as_double();
    no_samples_ = get_parameter("no_samples").as_int();

    if(no_samples_ > 0 )
    {
        samples_.poses = std::vector<geometry_msgs::msg::Pose>(no_samples_, geometry_msgs::msg::Pose());
    }
    else
    {
        RCLCPP_FATAL_STREAM(get_logger(), "Invalid Number of Samples requested: "<< no_samples_);
        return;
    }


    odom_sub_= create_subscription<nav_msgs::msg::Odometry>("walle_controller/odom", 10, std::bind(&OdometryMotionModel::odomCallback, this, _1));
    pose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("odometry_motion_model/samples", 10);
}


void OdometryMotionModel::odomCallback(const nav_msgs::msg::Odometry &odom)
{
    tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


    if(is_first_odom_)
    {
        last_odom_x_ = odom.pose.pose.position.x;
        last_odom_y_ = odom.pose.pose.position.y;
        last_odom_theta_ = yaw;
        samples_.header.frame_id = odom.header.frame_id;
        is_first_odom_ = false;
        return;
    }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryMotionModel>("odometry_motion_model");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}