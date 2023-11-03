#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/time_cache.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/shared_ptr.hpp>

geometry_msgs::TransformStamped robot_pose;
boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_ptr;
ros::Timer timer;
bool first_shot = true;

void meshGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_ptr)
{
  if (!first_shot)
  {
    return;
  }

  first_shot = false;
  ROS_INFO("Got robot pose");

  tf2::Stamped<tf2::Transform> robot_pose_tmp;
  tf2::fromMsg(*goal_ptr, robot_pose_tmp);
  robot_pose = tf2::toMsg(robot_pose_tmp);
  robot_pose.child_frame_id = "base_footprint";
  timer.start();
}

void publishTransform(const ros::TimerEvent& e)
{
  robot_pose.header.stamp = ros::Time::now();
  broadcaster_ptr->sendTransform(robot_pose);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_robot_pose");
  ros::NodeHandle nh;

  tf2::Stamped<tf2::Transform> robot_pose_tmp;
  robot_pose_tmp.setIdentity();

  broadcaster_ptr = boost::make_shared<tf2_ros::TransformBroadcaster>();
  ros::Subscriber mesh_pose = nh.subscribe("goal", 100, meshGoalCallback);
  ros::Duration duration(0.1);
  timer = nh.createTimer(duration, publishTransform, false, false);
  ros::spin();
}
