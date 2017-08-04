#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"

void mapCallback(const PointCloud::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> new_points = msg->points;
  obst_cloud += new_points;

  ros::NodeHandle nhpub_PC;
  ros:Publisher pub = nhpub_PC.advertise<PointCloud>("point_cloud",1);
}

int main(int argc, char ** argv)
{
  /*
   * Read incoming depth images translate them to PointClouds
   * [In] Depth image
   * [Out] Obstacle PointCloud
   * Note: Possibly use depth_image_proc nodelet to transform depth image to point cloud
  */
  pcl::PointCloud<pcl::PointXYZ> obst_cloud;
  obst_cloud.width = 3;
  obst_cloud.height = 1;

  ros::init(argc, argv, "map_listener");
  ros::NodeHandle hsub;
  ros::Subscriber sub = hsub.subscribe("points", 10, mapCallback)
}