#ifndef MAPPING_WITH_KNOWN_POSES
#define MAPPING_WITH_KNOWN_POSES

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace bumperbot_mapping
{

inline const double PRIOR_PROBABILITY = 0.5;
inline const double OCCUPANCY_PROBABILITY = 0.9;
inline const double FREE_PROBABILITY = 0.35;

struct Pose
{   
    Pose() = default;
    Pose(const int px, const int py) : x(px), y(py){}
    int x;
    int y;
};

unsigned int poseToCell(Pose pose, nav_msgs::msg::MapMetaData map_info);

Pose coordinatesToPose(const double px, const double py, nav_msgs::msg::MapMetaData map_info);

bool poseOnMap(Pose pose, nav_msgs::msg::MapMetaData map_info);

std::vector<Pose> bresenham(const Pose & start, const Pose & end);

std::vector<std::pair<Pose, double>> inverseSensorModel(const Pose & pose_robot, const Pose & pose_beam);

double prob_to_log_odds(double prob);
double log_odds_to_prob(double log_prob);


class MappingWithKnownPoses : public rclcpp::Node
{
public:
    MappingWithKnownPoses(const std::string & name);

private:
    nav_msgs::msg::OccupancyGrid map_;
    std::vector<double> probability_map_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    void scanCallback(const sensor_msgs::msg::LaserScan & scan);
    void timerCallback();


};
}
#endif