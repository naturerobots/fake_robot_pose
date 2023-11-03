cmake_minimum_required(VERSION 3.5)
project(robot_pose)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(static_robot_pose src/static_robot_pose.cpp)
ament_target_dependencies(static_robot_pose rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

#add_executable(odom_robot_pose src/odom_robot_pose.cpp)
#ament_target_dependencies(odom_robot_pose rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

install(TARGETS 
  static_robot_pose
#  odom_robot_pose
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

