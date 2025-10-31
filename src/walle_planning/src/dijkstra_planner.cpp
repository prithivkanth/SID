#include<vector>
#include<queue>

#include "walle_planning/dijkstra_planner.hpp"
#include "rmw/qos_profiles.h"

namespace walle_planning
{

void DijkstraPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  
}
nav_msgs::msg::Path DijkstraPlanner::plan(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
{
    std::vector<std::pair<int, int>> explore_directions = 
    {
        {-1,0}, {1,0}, {0,-1}, {0,1}
    };

   std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
   std::vector<GraphNode> visited_nodes;

   pending_nodes.push(worldToGrid(start));

    GraphNode active_node;
    while(!pending_nodes.empty() && rclcpp::ok())
    {
        active_node = pending_nodes.top();
        pending_nodes.pop();

        //goal found
        if(worldToGrid(goal) == active_node)
        {
            break;
        }

        //explore neighbors
        for (const auto &dir : explore_directions)
        {
            GraphNode new_node = active_node + dir;

            if (std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() &&
                poseOnMap(new_node) && map_->data.at(poseToCell(new_node)) < 99 && map_->data.at(poseToCell(new_node)) >=0)
                {
                    new_node.cost = active_node.cost + 1 + map_->data.at(poseToCell(new_node));
                    new_node.prev = std::make_shared<GraphNode>(active_node);
                    pending_nodes.push(new_node);
                    visited_nodes.push_back(new_node);
                }
        }

        visited_map_.data.at(poseToCell(active_node)) = 10; //blue
        map_pub_->publish(visited_map_);
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;
    while(active_node.prev && rclcpp::ok()) {
        geometry_msgs::msg::Pose last_pose = gridToWorld(active_node);
        geometry_msgs::msg::PoseStamped last_pose_stamped;
        last_pose_stamped.header.frame_id = map_->header.frame_id;
        last_pose_stamped.pose = last_pose;
        path.poses.push_back(last_pose_stamped);
        active_node = *active_node.prev;
    }
    std::reverse(path.poses.begin(), path.poses.end());
    return path;
}

bool DijkstraPlanner::poseOnMap(const GraphNode & node)
{
    return node.x < static_cast <int>(map_->info.width) && node.x >= 0 &&
            node.y < static_cast <int>(map_->info.height) && node.y >= 0;
}

GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
    int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
    int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);

    return GraphNode(grid_x,grid_y);

}

geometry_msgs::msg::Pose DijkstraPlanner::gridToWorld(const GraphNode & node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x * map_->info.resolution + map_->info.origin.position.x;
    pose.position.y = node.y * map_->info.resolution + map_->info.origin.position.y;
    return pose;
}

unsigned int DijkstraPlanner::poseToCell(const GraphNode & node)
{
    return map_-> info.width * node.y + node.x;
}

}


int main (int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<walle_planning::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

