#ifndef DIJKSTRA_PLANNER_HPP
#define DIJKSTRA_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace bumperbot_planning{

struct GraphNode
{
    int x;
    int y;
    int cost;
    std::shared_ptr<GraphNode> prev;

    // Constructor with given values
    GraphNode(int in_x, int in_y): x(in_x), y(in_y), cost(0)
    {
    }

    // Default Constructor 
    GraphNode(): GraphNode(0,0)
    {
        
    }

    // Redefining what > does
    bool operator>(const GraphNode & other) const
    {
        return cost > other.cost;
    }

    // Redefining what == does
    bool operator==(const GraphNode & other) const
    {
        return (x == other.x) && (y == other.y);
    }

    // Redefining what summation does
    GraphNode operator+(std::pair<int, int> const & other)
    {
        GraphNode result( x + other.first, y + other.second);
        return result;
    }
};

class DijsktraPlanner: public rclcpp::Node
{
public:
    DijsktraPlanner();

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::OccupancyGrid visited_map_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_);
    bool poseOnMap(const GraphNode & node);

    nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose & start_pose_, const geometry_msgs::msg::Pose & end_pose_);

    GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose);
    geometry_msgs::msg::Pose gridToWorld(const GraphNode & node);
    unsigned int cellToMapIndex(const GraphNode & node);
    
};
}

#endif // A_STAR_PLANNER_HPP