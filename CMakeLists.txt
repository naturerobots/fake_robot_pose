cmake_minimum_required(VERSION 2.8.3)
project(robot_pose)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(static_robot_pose src/static_robot_pose.cpp)
add_executable(odom_robot_pose src/odom_robot_pose.cpp)

add_dependencies(static_robot_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(odom_robot_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(static_robot_pose ${catkin_LIBRARIES})
target_link_libraries(odom_robot_pose ${catkin_LIBRARIES})

install(TARGETS static_robot_pose odom_robot_pose
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

