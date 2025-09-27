#include "walle_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"

namespace walle_planning
{
DijkstraPlanner::DijkstraPlanner() : Node("dijkstra_node")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", map_qos, std::bind(&DijkstraPlanner::mapCallback, this, std::placeholders::_1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, std::bind(&DijkstraPlanner::goalCallback, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/dijkstra/path", 10);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/dijkstra/visited_map", 10);
}

void DijkstraPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{

}
void DijkstraPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{

}

nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
{
    
}

}

