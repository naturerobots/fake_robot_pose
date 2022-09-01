#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/time_cache.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/shared_ptr.hpp>
#include <nav_msgs/Odometry.h>
#include <mutex>

boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_ptr;
size_t seq;
ros::Time last_call;
ros::Duration freq;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_ptr){

  if(ros::Time::now() - last_call < freq)
    return;
  else
    last_call = ros::Time::now();

  tf2::Stamped<tf2::Transform> robot_pose_tmp;
  tf2::fromMsg(odom_ptr->pose.pose, static_cast<tf2::Transform&>(robot_pose_tmp));
  geometry_msgs::TransformStamped robot_pose = tf2::toMsg(robot_pose_tmp);
  robot_pose.header.frame_id = "map";
  robot_pose.child_frame_id = "base_footprint";
  robot_pose.header.stamp = ros::Time::now();
  robot_pose.header.seq = seq++;
  broadcaster_ptr->sendTransform(robot_pose);
}

int main(int argc, char** argv){

  ros::init(argc, argv, "odom_robot_pose");
  ros::NodeHandle nh;

  // 30 Hz frequency
  freq = ros::Duration(1.0 / 30.0);
  last_call = ros::Time::now();

  seq = 0;
  broadcaster_ptr = boost::make_shared<tf2_ros::TransformBroadcaster>();
  ros::Subscriber sub = nh.subscribe("gazebo_odom", 100, odomCallback);
  ros::spin();

}
