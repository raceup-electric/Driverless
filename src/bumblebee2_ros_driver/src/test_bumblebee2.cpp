#include <memory>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include "bumblebee2_ros_driver/msg/stereo_image.hpp"

#include "bumblebee2_driver.h"

/* ROS LIBRARIES FOR PUBLISHER */
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

/* opencv img converter */
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

class TestBumblebee2 : public rclcpp::Node
{
public:
  TestBumblebee2 ()
  : Node("test_bumblebee2")
  {
    subscription_ = this->create_subscription<bumblebee2_ros_driver::msg::StereoImage>(
      "stereo_image", 10, std::bind(&TestBumblebee2 ::topicCallback, this, _1));

     publisher_camera_left_  = this->create_publisher<sensor_msgs::msg::Image>("/camera/left", 10);
     publisher_camera_right_  = this->create_publisher<sensor_msgs::msg::Image>("/camera/right", 10);

    // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", 10);
    
    // cv::namedWindow("left");
    // cv::namedWindow("right");
  }

private:

  bool isValueNearCenter(int value, int center, int thresh){

    if(value > (center - thresh) && value < (center + thresh) )
        return true;
    else
        return false;

  }

  bool isSimilarColor(const cv::Vec3b &color, const cv::Vec3b pixel, int T){

    if(isValueNearCenter(pixel[0], color[0], T) && isValueNearCenter(pixel[2], color[2], T) && (pixel[2], color[2], T) )
                return true;
            else
                return false;
  }

  cv::Mat createMask(cv::Vec3b blue, cv::Vec3b yellow, const cv::Mat &image, int T){

    cv::Mat mask(image.rows, image.cols, CV_8U);
    cv::Vec3b bgrVector;
    cv::Vec3b black(0,0,0);

    for(int row = 0; row < image.rows; row++){
        for(int col = 0; col < image.cols; col++){
            bgrVector = image.at<cv::Vec3b>(row,col);

            // check if all the BGR pixel values are within a threshold of the given avg
            if( isSimilarColor(blue, bgrVector, T) || isSimilarColor(yellow, bgrVector, T) )
                mask.at<uchar>(row, col) = 255;
            else
                mask.at<uchar>(row, col) = 0;              
        }
    }
    return mask;
  }

  void topicCallback(const bumblebee2_ros_driver::msg::StereoImage::SharedPtr msg)
  {
    
    
    cv::Mat_<uchar> left_img(msg->height, msg->width, msg->left_data.data()),
                        right_img(msg->height, msg->width, msg->right_data.data());
                        
    cv::Mat_<cv::Vec3b> left_img_rgb(left_img.rows, left_img.cols),
                        right_img_rgb(right_img.rows, right_img.cols);

    bumblebee2Debayer(left_img, left_img_rgb);
    bumblebee2Debayer(right_img, right_img_rgb);


    cv::Vec3i bgrYellowCones(117, 227, 246);
    cv::Vec3i bgrBlueCones(227, 142, 79);

    // cv::Mat left_cropped_image = left_img_rgb(cv::Rect(0, 0, left_img_rgb.cols, left_img_rgb.rows - 100));
    // cv::Mat right_cropped_image = right_img_rgb(cv::Rect(0, 0, right_img_rgb.cols, right_img_rgb.rows - 100));

    // double alpha = 0.75; /*< Simple contrast control */
    // int beta = -40; /*< Simple brightness control */
 

    // for( int y = 0; y < left_cropped_image.rows; y++ ) {
    //   for( int x = 0; x < left_cropped_image.cols; x++ ) {
    //     for( int c = 0; c < left_cropped_image.channels(); c++ ) {
    //       left_cropped_image.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>( alpha*left_img_rgb.at<cv::Vec3b>(y,x)[c] + beta );
    //       right_cropped_image.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>( alpha*right_img_rgb.at<cv::Vec3b>(y,x)[c] + beta );
    //     }
    //   }
    // }


    // left_cropped_image = usm(left_cropped_image, 0.8, 12., 1.);
    // right_cropped_image = usm(right_cropped_image, 0.8, 12., 1.);

    cv::Mat left_grey_image = createMask(bgrYellowCones, bgrBlueCones, left_img_rgb, 30);
    cv::Mat right_grey_image = createMask(bgrYellowCones, bgrBlueCones, right_img_rgb, 30);

    // added to publish topics: /camera/right /camera/left
    sensor_msgs::msg::Image::SharedPtr left_camera_image = cv_bridge::CvImage(msg->header, "mono8", left_grey_image).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr right_camera_image = cv_bridge::CvImage(msg->header, "mono8", right_grey_image).toImageMsg();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: image", msg->header.);

    publisher_camera_left_->publish(*left_camera_image.get());
    publisher_camera_right_->publish(*right_camera_image.get());

    //printf ( "%ld\n", time_stamp );

    // show images for debugging
    cv::imshow("left", left_grey_image);
    cv::imshow("right", right_grey_image);

    int key = cv::waitKey(1);

    /* if (key == 97) {
        // Save the image
        cv::imwrite("left1.jpg", left_cropped_image);
        cv::imwrite("right1.jpg", right_cropped_image);
        // cout << "Image captured and saved as captured_image.jpg" << endl;
    } else if (key == 98) {
        // Save the image
        cv::imwrite("left2.jpg", left_cropped_image);
        cv::imwrite("right2.jpg", right_cropped_image);
        // cout << "Image captured and saved as captured_image.jpg" << endl;
    } else {
        // cout << "No key was pressed. Exiting..." << endl;
    } */
  }
  
  rclcpp::Subscription<bumblebee2_ros_driver::msg::StereoImage>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_camera_right_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestBumblebee2>());
  rclcpp::shutdown();
  return 0;
}
