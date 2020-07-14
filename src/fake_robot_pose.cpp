#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/convert.h>
#include <tf/transform_datatypes.h>
#include <tf/time_cache.h>
#include <tf/transform_broadcaster.h>
#include <boost/shared_ptr.hpp>

tf::StampedTransform robot_pose;
boost::shared_ptr<tf::TransformBroadcaster> broadcaster_ptr;
ros::Timer timer;
bool first_shot = true;

void meshGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_ptr){
  if(!first_shot){
    return;
  }

  first_shot = false;
  ROS_INFO("Got robot pose");
  tf::Stamped<tf::Pose> pose;
  tf::poseStampedMsgToTF(*goal_ptr, pose);
  robot_pose = tf::StampedTransform(pose, ros::Time::now(), "map", "base_footprint");
  timer.start();
}

void publishTransform(const ros::TimerEvent& e){
  robot_pose.stamp_ = ros::Time::now();
  broadcaster_ptr->sendTransform(robot_pose);
}


int main(int argc, char** argv){

  ros::init(argc, argv, "fake_robot_pose");
  ros::NodeHandle nh;

  robot_pose.setIdentity();
  broadcaster_ptr = boost::make_shared<tf::TransformBroadcaster>();
  ros::Subscriber mesh_pose = nh.subscribe("goal", 100, meshGoalCallback);
  ros::Duration duration(0.1);
  timer = nh.createTimer(duration, publishTransform, false, false);
  ros::spin();

}
