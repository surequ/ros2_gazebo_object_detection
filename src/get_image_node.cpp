#include <cstdio>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"  // 受け取るメッセージ型
#include "sensor_msgs/msg/image.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("get_image_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera1/image_raw", 10,
      std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }
//sensor_msgs/msg/Image
private:
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat img = cv_bridge::toCvShare(msg,"bgr8")->image;
    cv::imshow("Img",img);
    cv::waitKey(1);
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
