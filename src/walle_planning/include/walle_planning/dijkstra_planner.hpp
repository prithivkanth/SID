#ifndef DIJKSTRA_PLANNER_HPP
#define DIJKSTRA_PLANNER_HPP

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace walle_planning
{

struct GraphNode
{
    int x;
    int y;
    int cost;
    std::shared_ptr<GraphNode> prev;

    GraphNode() : GraphNode(0,0){}
    
    GraphNode(int in_x, int in_y) : x(in_x), y(in_y), cost(0){}
    
    bool operator>(const GraphNode & other ) const 
    {
        return cost > other.cost;
    }

    bool operator==(const GraphNode & other ) const 
    {
        return x == other.x && y == other.y;
    }

    GraphNode operator+(std::pair<int, int> const & other)
    {
        GraphNode res(x + other.first, y + other.second);
        return res;
    }
    
};

class DijkstraPlanner : public nav2_core::GlobalPlanner
{
public:
    DijkstraPlanner() = default;
    ~DijkstraPlanner() = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name, 
        std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

private:

    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D *costmap_;
    std::string global_frame_,name_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);

    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);

    bool poseOnMap(const GraphNode & node);
    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
    unsigned int poseToCell(const GraphNode & node);


};
}

#endif //DIJKSTRA_PLANNER_HPP