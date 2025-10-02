#include "walle_motion/pd_motion_planner.hpp"


namespace walle_motion
{
PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner_node"),
  kp_(2.0), kd_(0.1), step_size_(0.2), max_linear_velocity_(0.3),
  max_angular_velocity_(1.0), prev_angular_error_(0.0), prev_linear_error_(0.0)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declare_parameter<double>("kp", kp_);
  declare_parameter<double>("kd", kd_);
  declare_parameter<double>("step_size", step_size_);
  declare_parameter<double>("max_linear_velocity", max_linear_velocity_);
  declare_parameter<double>("max_angular_velocity", max_angular_velocity_);
  kp_ = get_parameter("kp").as_double();
  kd_ = get_parameter("kd").as_double();
  step_size_ = get_parameter("step_size").as_double();
  max_linear_velocity_ = get_parameter("max_linear_velocity").as_double();
  max_angular_velocity_ = get_parameter("max_angular_velocity").as_double();

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/a_star/path", 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));
        
  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);
  control_loop_ = create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));
}

void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
  global_plan_ = *path;
}

void PDMotionPlanner::controlLoop()
{
  if(global_plan_.poses.empty()){
    return;
  }

  // Get the robot's current pose in the odom frame
  geometry_msgs::msg::TransformStamped robot_pose;
  try {
    robot_pose = tf_buffer_->lookupTransform(
      "odom", "base_footprint", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "frame id Robot Pose: %s", robot_pose.header.frame_id.c_str());
  RCLCPP_INFO(get_logger(), "frame id GLOBAL Plan: %s", global_plan_.header.frame_id.c_str());
}

geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::PoseStamped next_pose = global_plan_.poses.back();

  for (auto pose_it = global_plan_.poses.rbegin(); pose_it != global_plan_.poses.rend(); ++pose_it)
  {
    double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
    double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
    double distance = std::sqrt(dx*dx + dy*dy);

    if (distance > step_size_)
    {
      next_pose = *pose_it;
    }
    else
    {
      break;
    }
  }

  return next_pose;
}

bool PDMotionPlanner::transformPlan(const std::string & frame)
{
  if (global_plan_.header.frame_id == frame)
  {
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;

  try
  {
    transform = tf_buffer_->lookupTransform(frame, global_plan_.header.frame_id, tf2::TimePointZero);
  }
  catch(tf2::ExtrapolationException & ex)
  {
    RCLCPP_ERROR_STREAM(get_logger(), "couldn't transform plan from frame "<<global_plan_.header.frame_id<<" to frame " << frame);
    return false;
  }
  for (auto & pose : global_plan_.poses)
  {
    tf2::doTransform(pose, pose, transform);
  }

  global_plan_.header.frame_id = frame;
  return true;
}

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<walle_motion::PDMotionPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
