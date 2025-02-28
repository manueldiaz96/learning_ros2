cmake_minimum_required(VERSION 3.8)
project(bumperbot_utils)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(bumperbot_interfaces REQUIRED)


include_directories(include)

add_executable(path_publisher src/path_publisher.cpp)
ament_target_dependencies(path_publisher rclcpp geometry_msgs nav_msgs tf2 tf2_ros bumperbot_interfaces)

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  ## Which executables are put into the insall folder
  path_publisher
  DESTINATION lib/${PROJECT_NAME} ## Variable for the local project name
)


ament_package()
