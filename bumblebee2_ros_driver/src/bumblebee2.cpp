#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "bumblebee2_ros_driver/msg/stereo_image.hpp"

#include "bumblebee2_driver.h"
#include "rt_utils.h"

using namespace std::chrono_literals;

class Ros2Bumblebee2Driver : public Bumblebee2Driver
{
public:
  virtual ~Ros2Bumblebee2Driver(){};
protected:
  virtual uint64_t getTime()
  {
    return ros_clock_.now().nanoseconds();
  }
  rclcpp::Clock ros_clock_;
};

class Bumblebee2 : public rclcpp::Node
{
  public:

    Bumblebee2();
    virtual ~Bumblebee2(){};

    void run();

  private:
    Ros2Bumblebee2Driver driver_;

    rclcpp::Publisher<bumblebee2_ros_driver::msg::StereoImage>::SharedPtr publisher_;
    bumblebee2_ros_driver::msg::StereoImage message_;

    void declareParameters();
};

Bumblebee2 :: Bumblebee2() : Node("bumblebee2")
{
  declareParameters();

  driver_.openDevice();

  driver_.setShutter( this->get_parameter("shutter").as_int() );
  driver_.setGain( this->get_parameter("gain").as_int() );

  setupRealTime(this->get_parameter("sched_priority").as_int());

  publisher_ = this->create_publisher<bumblebee2_ros_driver::msg::StereoImage>("stereo_image", 10);

  std::cout<<"Camera shutter : "<<driver_.getShutter()<<std::endl;
  std::cout<<"Camera gain : "<<driver_.getGain()<<std::endl;

  run();
}

void Bumblebee2 :: declareParameters()
{
  this->declare_parameter<int>("shutter", 379);
  this->declare_parameter<int>("gain", 118);
  this->declare_parameter<int>("sched_priority", 99);
}

void Bumblebee2 :: run()
{
  while( rclcpp::ok() )
  {
    bumblebee2_ros_driver::msg::StereoImage::UniquePtr stereo_msg(new bumblebee2_ros_driver::msg::StereoImage());
    stereo_msg->header.frame_id = "stereo_camera";
    stereo_msg->width = driver_.width();
    stereo_msg->height = driver_.height();
    stereo_msg->encoding = "mono8";
    stereo_msg->is_bigendian = false;
    stereo_msg->step = stereo_msg->width;
    size_t size = stereo_msg->width * stereo_msg->height;
    stereo_msg->left_data.resize(size);
    stereo_msg->right_data.resize(size);
    uint64_t time_stamp; 
    driver_.acquireFrame( stereo_msg->left_data.data(), stereo_msg->right_data.data(), time_stamp );
    rclcpp::Time ros_stamp(time_stamp);
    stereo_msg->header.stamp = ros_stamp;

    publisher_->publish(std::move(stereo_msg));
  }
  
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bumblebee2>());
  rclcpp::shutdown();
  return 0;
}
