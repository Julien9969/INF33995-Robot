// #include <nav_msgs/msg/odometry.hpp>
#include "pcl_conversions/pcl_conversions.h"


void cloud_callback(const sensor_msgs::msg::PointCloud2 &msg)
{
  printf("converting PCL...\n");
  // PCL still uses boost::shared_ptr internally
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
    std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

  // This will convert the message into a pcl::PointCloud
  // pcl::fromROSMsg(msg, *cloud);
}