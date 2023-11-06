#include <chrono>
#include <memory>
#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/time_cache.h>
#include <tf2/transform_datatypes.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class StaticRobotPose : public rclcpp::Node
{
public:
  StaticRobotPose()
    : Node("static_robot_pose")
    , broadcaster_(this)
    , has_received_mesh_goal_pose_(false)
    , timer_(create_wall_timer(0.1s, std::bind(&StaticRobotPose::publishTransform, this)))
  {
    subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal", 100, std::bind(&StaticRobotPose::meshGoalCallback, this, _1));
  }

private:
  void meshGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goal_ptr)
  {
    if (has_received_mesh_goal_pose_)
    {
      return;
    }

    has_received_mesh_goal_pose_ = true;
    RCLCPP_INFO(get_logger(), "Got robot pose");

    robot_pose_.header = goal_ptr->header;
    robot_pose_.transform.rotation = goal_ptr->pose.orientation;
    robot_pose_.transform.translation.x = goal_ptr->pose.position.x;
    robot_pose_.transform.translation.y = goal_ptr->pose.position.y;
    robot_pose_.transform.translation.z = goal_ptr->pose.position.z;
    robot_pose_.header = goal_ptr->header;
    robot_pose_.child_frame_id = "base_footprint";
  }

  void publishTransform()
  {
    if (!has_received_mesh_goal_pose_)
    {
      return;
    }
    robot_pose_.header.stamp = get_clock()->now();
    broadcaster_.sendTransform(robot_pose_);
  }

  geometry_msgs::msg::TransformStamped robot_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  tf2_ros::TransformBroadcaster broadcaster_;
  bool has_received_mesh_goal_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticRobotPose>());
  rclcpp::shutdown();
  return 0;
}