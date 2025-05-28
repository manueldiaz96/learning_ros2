#include "bumperbot_mapping/mapping_with_known_poses.hpp"

#include "tf2/utils.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.hpp>

namespace bumperbot_mapping
{
Pose coordinatesToPose(const double px, const double py, nav_msgs::msg::MapMetaData map_info)
{
    Pose robot_pose;
    
    robot_pose.x = std::round((px - map_info.origin.position.x) / map_info.resolution);
    robot_pose.y = std::round((py - map_info.origin.position.y) / map_info.resolution);

    return robot_pose;
}

bool poseOnMap(Pose pose, nav_msgs::msg::MapMetaData map_info)
{
    if(pose.x > static_cast<int>(map_info.width) || pose.x < 0){
        return false;
    }
    else if (pose.y > static_cast<int>(map_info.height) || pose.y < 0){
        return false;
    }
    else{
        return true;
    }
}

unsigned int poseToCell(Pose pose, nav_msgs::msg::MapMetaData map_info)
{
    unsigned int robot_cell;
    robot_cell = map_info.height * pose.y + pose.x;

    return robot_cell;
}

std::vector<Pose> bresenham(const Pose & start, const Pose & end)
{
    // Implementation of Bresenham's line drawing algorithm
    // See en.wikipedia.org/wiki/Bresenham's_line_algorithm
    std::vector<Pose> line;
    int dx = end.x - start.x;
    int dy = end.y - start.y;
    int xsign = dx > 0 ? 1 : -1;
    int ysign = dy > 0 ? 1 : -1;
    dx = std::abs(dx);
    dy = std::abs(dy);
    int xx, xy, yx, yy;

    if(dx > dy)
    {
        xx = xsign;
        xy = 0;
        yx = 0;
        yy = ysign;
    }
    else
    {
        int tmp = dx;
        dx = dy;
        dy = tmp;
        xx = 0;
        xy = ysign;
        yx = xsign;
        yy = 0;
    }

    int D = 2 * dy - dx;
    int y = 0;
    line.reserve(dx + 1);

    for (int i = 0; i < dx + 1; i++)
    {
        line.emplace_back(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy));
        if(D >= 0)
        {
            y++;
            D -= 2 * dx;
        }
        D += 2 * dy;
    }
    return line;
}

std::vector<std::pair<Pose, double>> inverseSensorModel(const Pose & pose_robot, const Pose & pose_beam)
{
    std::vector<Pose> laser_beam = bresenham(pose_robot, pose_beam);
    std::vector<std::pair<Pose, double>> occupancies;

    for(size_t i=0; i < laser_beam.size()-1; i++)
    {
        occupancies.emplace_back(std::pair<Pose, double>(laser_beam.at(i), FREE_PROBABILITY));
    }

    // Last position should be where the laser hit
    occupancies.emplace_back(std::pair<Pose, double>(laser_beam.back(), OCCUPANCY_PROBABILITY));

    return occupancies;
}

double prob_to_log_odds(double prob)
{
    return std::log(prob / (1 - prob));
}

double log_odds_to_prob(double log_prob)
{
    return 1 - (1 / (1 + std::exp(log_prob)));
}

MappingWithKnownPoses::MappingWithKnownPoses(const std::string & name) : Node(name)
{
    declare_parameter<double>("width", 50.0);
    declare_parameter<double>("height", 50.0);
    declare_parameter<double>("resolution", 0.1);

    double width = get_parameter("width").as_double();
    double height = get_parameter("height").as_double();
    map_.info.resolution = get_parameter("resolution").as_double();

    map_.info.height = std::round(height / map_.info.resolution);
    map_.info.width = std::round(width / map_.info.resolution);

    map_.info.origin.position.x = -std::round(width / 2.);
    map_.info.origin.position.y = -std::round(height / 2.);

    map_.header.frame_id = "odom";

    map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, -1);

    probability_map_ = std::vector<double>(map_.info.width * map_.info.height, prob_to_log_odds(PRIOR_PROBABILITY));

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(
                                                                                        &MappingWithKnownPoses::scanCallback, 
                                                                                        this, 
                                                                                        std::placeholders::_1
                                                                                    ));
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(
                                                                   &MappingWithKnownPoses::timerCallback, 
                                                                   this
                                                                ));
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void MappingWithKnownPoses::scanCallback(const sensor_msgs::msg::LaserScan & scan)
{
    geometry_msgs::msg::TransformStamped t ;

    try {
        t = tf_buffer_->lookupTransform(map_.header.frame_id, scan.header.frame_id, tf2::TimePointZero);
    } catch(const tf2::TransformException & e){
        RCLCPP_ERROR(get_logger(), "Unable to transform between odom and base_footprint");
        return;
    }

    Pose robot_pose = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, map_.info);

    if(!poseOnMap(robot_pose, map_.info))
    {
        RCLCPP_ERROR(get_logger(), "Robot is outside of the map! Robot on coords: {x: %i, y: %i}", robot_pose.x, robot_pose.y);
        return;
    }

    tf2::Quaternion q(t.transform.rotation.x,
                      t.transform.rotation.y,
                      t.transform.rotation.z,
                      t.transform.rotation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    std::vector<Pose> occupied_cells;
    
    for (size_t i=0; i < scan.ranges.size(); i++)
    {   
        double range = scan.ranges.at(i);
        double angle = (scan.angle_increment * i) + scan.angle_min + yaw; 

        if (std::isinf(range) || range > scan.range_max || range < scan.range_min) {
            continue;
        }

        double x = (range * std::cos(angle)) + t.transform.translation.x;
        double y = (range * std::sin(angle)) + t.transform.translation.y;

        Pose range_pose = coordinatesToPose(x, y, map_.info);

        if(poseOnMap(range_pose, map_.info)){
            // unsigned int range_cell = poseToCell(range_pose, map_.info);
            // map_.data.at(range_cell) = 100;

            std::vector<std::pair<Pose, double>> occupancies;
            occupancies = inverseSensorModel(robot_pose, range_pose);

            for (auto cell_occ : occupancies)
            {
                unsigned int cell_idx = poseToCell(cell_occ.first, map_.info);
                probability_map_.at(cell_idx) += prob_to_log_odds(cell_occ.second) - prob_to_log_odds(PRIOR_PROBABILITY);
            }

        }
    }

    // unsigned int robot_cell = poseToCell(robot_pose, map_.info);

    // map_.data.at(robot_cell) = 100;

}

void MappingWithKnownPoses::timerCallback()
{
    map_.header.stamp = get_clock()->now();
    std::transform(probability_map_.begin(), probability_map_.end(), map_.data.begin(), [](double value){
        return log_odds_to_prob(value) * 100;
    });
    map_pub_->publish(map_);
}

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_mapping::MappingWithKnownPoses>("mapping_with_known_poses");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}