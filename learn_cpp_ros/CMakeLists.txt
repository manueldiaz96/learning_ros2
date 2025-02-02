cmake_minimum_required(VERSION 3.8)
project(learn_cpp_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Executable Files
# Executable Files
add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp std_msgs)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp std_msgs example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server_node.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(reset_number_client src/counter_reset_client.cpp)
ament_target_dependencies(reset_number_client rclcpp example_interfaces)

add_executable(hw_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher rclcpp example_interfaces my_robot_interfaces)

add_executable(compute_area_server_node src/compute_area_server_node.cpp)
ament_target_dependencies(compute_area_server_node rclcpp example_interfaces my_robot_interfaces)

add_executable(battery_node src/battery_led_exercise/battery_node.cpp)
ament_target_dependencies(battery_node rclcpp my_robot_interfaces)

add_executable(led_node src/battery_led_exercise/led_node.cpp)
ament_target_dependencies(led_node rclcpp my_robot_interfaces)

add_executable(robot_news_publisher src/robot_news_station/robot_news_publisher.cpp)
ament_target_dependencies(robot_news_publisher rclcpp std_msgs)

add_executable(smartphone src/robot_news_station/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp std_msgs)

## Where they should be installed

install(TARGETS
  number_counter
  number_publisher
  add_two_ints_server
  add_two_ints_client
  add_two_ints_client_no_oop
  reset_number_client
  hw_status_publisher
  compute_area_server_node
  battery_node
  led_node
  robot_news_publisher 
  smartphone
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
