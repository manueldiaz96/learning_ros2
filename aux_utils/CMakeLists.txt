cmake_minimum_required(VERSION 3.8)
project(aux_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(twist_stamped_publisher src/twist_stamped_publisher.cpp)
ament_target_dependencies(twist_stamped_publisher rclcpp geometry_msgs std_msgs)

install(TARGETS
  twist_stamped_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
