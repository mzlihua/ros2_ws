cmake_minimum_required(VERSION 3.8)
project(go2_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(go2_driver src/go2_driver.cpp)
target_compile_features(go2_driver PUBLIC cxx_std_17)
# NOTE: this distro ships ament_cmake_target_dependencies without the
# ament_target_dependencies macro, so link the ament-exported targets directly.
target_link_libraries(go2_driver PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  sensor_msgs::sensor_msgs
  trajectory_msgs::trajectory_msgs
  tf2_ros::tf2_ros
)

install(TARGETS go2_driver
  DESTINATION lib/${PROJECT_NAME})

# Scenery world, launch files, bridge config and the URDF->SDF converter
install(DIRECTORY worlds launch config scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
