#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/LinearMath/Vector3.h>

void poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  Vector3 pos;
  pos[0] = msg->pose.pose.position.x;
  pos[1] = msg->pose.pose.position.y;
  pos[2] = msg->pose.pose.position.z;


}

int main(int argc, char ** argv)
{
  /* 
   * Create depth image based on camera orientation and features
   * * [In] Drone pose
   * * [Out] Depth image
  */
  ros::init(argc, argv, "pose_listener");
  ros::NodeHandle hsub;
  ros::Subscriber sub = hsub.subscribe("pose", 10, poseCallback)
  


}