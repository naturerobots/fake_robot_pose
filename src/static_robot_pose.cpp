#include <chrono>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/time_cache.h>
#include <tf2/transform_datatypes.h>

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr node_ptr;
geometry_msgs::msg::TransformStamped robot_pose;
std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_ptr;
rclcpp::TimerBase::SharedPtr timer_ptr;
bool has_received_mesh_goal_pose = false;

void meshGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goal_ptr)
{
  if (has_received_mesh_goal_pose)
  {
    return;
  }

  has_received_mesh_goal_pose = true;
  // RCLCPP_INFO(node_ptr->get_logger(), "Got robot pose"); TODO segfaults...

  robot_pose.header = goal_ptr->header;
  robot_pose.transform.rotation = goal_ptr->pose.orientation;
  robot_pose.transform.translation.x = goal_ptr->pose.position.x;
  robot_pose.transform.translation.y = goal_ptr->pose.position.y;
  robot_pose.transform.translation.z = goal_ptr->pose.position.z;
  robot_pose.header = goal_ptr->header;
  robot_pose.child_frame_id = "base_footprint";
}

void publishTransform()
{
  if (!has_received_mesh_goal_pose)
  {
    return;
  }
  robot_pose.header.stamp = node_ptr->get_clock()->now();
  broadcaster_ptr->sendTransform(robot_pose);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  node_ptr = rclcpp::Node::make_shared("static_robot_pose");
  broadcaster_ptr = std::make_shared<tf2_ros::TransformBroadcaster>(*node_ptr);
  timer_ptr = node_ptr->create_wall_timer(0.1s, &publishTransform);
  const auto subscription =
      node_ptr->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 100, meshGoalCallback);
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}