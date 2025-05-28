#include <bumperbot_planning/a_star_planner.hpp>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <queue>

namespace bumperbot_planning
{


AStarPlanner::AStarPlanner() : Node("a_star_node")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/a_star/visited_map", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/a_star/path", 10);


    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                                                                 "/map", 
                                                                 map_qos,
                                                                 std::bind(
                                                                            &AStarPlanner::mapCallback, 
                                                                            this, 
                                                                            std::placeholders::_1
                                                                         )                                        
                                                                );

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
                                                                      "/goal_pose",
                                                                      10,
                                                                      std::bind(
                                                                        &AStarPlanner::goalCallback,
                                                                        this,
                                                                        std::placeholders::_1
                                                                      )   
                                                                    );
}

void AStarPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    map_ = map;
    visited_map_.header.frame_id = map_->header.frame_id;
    visited_map_.info = map_->info;

    visited_map_.data = std::vector<int8_t>( visited_map_.info.height * visited_map_.info.width, -1);
}

bool AStarPlanner::poseOnMap(const GraphNode & node)
{
    return node.x >= 0 && node.x < static_cast<int>(map_->info.width) &&
            node.y >= 0 && node.y < static_cast<int>(map_->info.height);
}

unsigned int AStarPlanner::cellToMapIndex(const GraphNode & node)
{
    return node.x + (node.y * map_->info.width);
}


GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose)
{
   int grid_x = static_cast<int>((pose.position.x - map_->info.origin.position.x) / map_->info.resolution);
   int grid_y = static_cast<int>((pose.position.y - map_->info.origin.position.y) / map_->info.resolution);

   return GraphNode(grid_x, grid_y);
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorld(const GraphNode & node){
    geometry_msgs::msg::Pose pose;

    pose.position.x = (node.x * map_->info.resolution) + map_->info.origin.position.x;
    pose.position.y = (node.y * map_->info.resolution) + map_->info.origin.position.y;

    return pose;

}

double AStarPlanner::manhattanDistance(const GraphNode & current, const GraphNode & goal)
{
    return abs(current.x - goal.x) + abs(current.y - goal.y);
}


void AStarPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    if(!map_)
{
        RCLCPP_ERROR(get_logger(), "Map not available!");
        return;
    }

    visited_map_.data = std::vector<int8_t>( visited_map_.info.height * visited_map_.info.width, -1);
    
    geometry_msgs::msg::TransformStamped map_to_base_tf;

    try{
        map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero);
    }catch(const tf2::TransformException &ex){
        RCLCPP_ERROR(get_logger(), "Transform from map to base_footprint not available!");
        return;
    }

    geometry_msgs::msg::Pose map_to_base_pose;
    map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
    map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
    map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

    auto path = plan(map_to_base_pose, pose->pose);
    if(!path.poses.empty())
    {
        RCLCPP_INFO(get_logger(), "Shortest path found!");
        path_pub_->publish(path);
    }else{
        RCLCPP_ERROR(get_logger(), "No path found to goal!");
    }
}

nav_msgs::msg::Path AStarPlanner::plan(const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose)
{
    std::vector<std::pair<int, int>> explore_directions = {
        {-1, 0}, // Left 
        {1, 0},  // Right
        {0, -1}, // Down
        {0, 1}   // Up
    };

    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;
    std::vector<GraphNode> visited_nodes;

    GraphNode start_node = worldToGrid(start_pose);
    GraphNode end_node = worldToGrid(end_pose);

    start_node.heuristic = manhattanDistance(start_node, end_node);
    pending_nodes.push(start_node);

    GraphNode active_node;

    while(!pending_nodes.empty() && rclcpp::ok())
    {
        active_node = pending_nodes.top();
        pending_nodes.pop();

        if(end_node == active_node){
            break;
        }

        for (const auto & dir : explore_directions){
            GraphNode new_node = active_node + dir;

               // Check if new_node has already been visited
            if(std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() && 
               // If it is on the map
               poseOnMap(new_node) && 
               // and if it is free
               map_->data.at(cellToMapIndex(new_node)) == 0)
            {
                new_node.cost = active_node.cost + 1;
                new_node.heuristic = manhattanDistance(new_node, end_node);
                new_node.prev = std::make_shared<GraphNode>(active_node);
                pending_nodes.push(new_node);
                visited_nodes.push_back(new_node);
            }
        }

        visited_map_.data.at(cellToMapIndex(active_node)) = -106;
        map_pub_-> publish(visited_map_);
    }
    
    nav_msgs::msg::Path path;
    path.header.frame_id = map_->header.frame_id;
    while (active_node.prev && rclcpp::ok())
    {
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




}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_planning::AStarPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}