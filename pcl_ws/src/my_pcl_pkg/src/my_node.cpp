#include <cstdio>
#include "my_pcl_pkg/convert.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("my_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "rgbd/image_raw/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%u'", msg.data);
      
      cloud_callback(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;


  printf("STARTING NODE PCL\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;

}
