#include <cstdio>
#include "my_pcl_pkg/convert.h"

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;








using std::placeholders::_1;


class PCLTransformer : public rclcpp::Node
{
  public:
    PCLTransformer()
    : Node("my_node")
    {
      // Store frame names in variables that will be used to
      // compute transformations
      std::string fromFrameRel = "diff_drive_2/depth_camera_joint_optical_link";
      std::string toFrameRel = "odom";
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "rgbd/image_raw/points", 10, std::bind(&PCLTransformer::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2 & msg) const
    {
      for(int i = 0; i < 20; i ++) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%u'", msg.data[i]);
      }
      
      cloud_callback(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame_;
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;


  printf("STARTING NODE PCL\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLTransformer>());
  rclcpp::shutdown();
  return 0;

}
