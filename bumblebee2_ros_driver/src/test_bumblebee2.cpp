#include <memory>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include "bumblebee2_ros_driver/msg/stereo_image.hpp"

#include "bumblebee2_driver.h"

using std::placeholders::_1;

class TestBumblebee2 : public rclcpp::Node
{
public:
  TestBumblebee2 ()
  : Node("test_bumblebee2")
  {
    subscription_ = this->create_subscription<bumblebee2_ros_driver::msg::StereoImage>(
      "stereo_image", 10, std::bind(&TestBumblebee2 ::topicCallback, this, _1));
    
    cv::namedWindow("left");
    cv::namedWindow("right");
  }

private:
  void topicCallback(const bumblebee2_ros_driver::msg::StereoImage::SharedPtr msg) const
  {
    
    
    cv::Mat_<uchar> left_img(msg->height, msg->width, msg->left_data.data()),
                        right_img(msg->height, msg->width, msg->right_data.data());
                        
    cv::Mat_<cv::Vec3b> left_img_rgb(left_img.rows, left_img.cols),
                        right_img_rgb(right_img.rows, right_img.cols);

    bumblebee2Debayer(left_img, left_img_rgb);
    bumblebee2Debayer(right_img, right_img_rgb);

    //printf ( "%ld\n", time_stamp );
    cv::imshow("left", left_img_rgb);
    cv::imshow("right", right_img_rgb);
    cv::waitKey(10);
  }
  
  rclcpp::Subscription<bumblebee2_ros_driver::msg::StereoImage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestBumblebee2>());
  rclcpp::shutdown();
  return 0;
}
