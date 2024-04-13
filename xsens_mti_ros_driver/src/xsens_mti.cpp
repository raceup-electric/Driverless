#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "rt_utils.h"
#include "xsens_mti_driver.h"

using namespace std::chrono_literals;

class XSensMTi : public rclcpp::Node
{
  public:

    XSensMTi();
    virtual ~XSensMTi();

  private:
    XSensMTiDriver driver_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    sensor_msgs::msg::Imu message_;

    void driverCallback( double *data, uint64_t time_stamp );
    void timeoutCallback();

    void declareParameters();
    void calculateOrientation(double *data);
};


XSensMTi :: XSensMTi() : Node("xsens_mti")
{
  message_.orientation_covariance = {0};
  message_.angular_velocity_covariance = {0};
  message_.linear_acceleration_covariance = {0};
  
  declareParameters();
  
  driver_.setSampleFrequency( this->get_parameter("sample_frequency").as_int() );
  driver_.enableCalibratedData( this->get_parameter("calibrated").as_bool() );
  driver_.enableLowLatencySerial( this->get_parameter("low_latency_serial").as_bool() );
  
  setupRealTime(this->get_parameter("sched_priority").as_int());
  
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  driver_.setDataCallback(std::bind(&XSensMTi::driverCallback, this, std::placeholders::_1, std::placeholders::_2)); 
  driver_.setTimeoutCallback(std::bind(&XSensMTi::timeoutCallback, this)); 
  driver_.startAcquisition();
}

XSensMTi :: ~XSensMTi()
{
  driver_.stopAcquisition();
}

void XSensMTi :: driverCallback( double *data, uint64_t time_stamp )
{
  if( rclcpp::ok() )
  {
    message_.header.stamp = this->get_clock()->now();
    message_.header.frame_id = "base_link";
    
    // Direct measurements
    message_.linear_acceleration.x = data[0];
    message_.linear_acceleration.y = data[1];
    message_.linear_acceleration.z = data[2];
    
    message_.angular_velocity.x = data[3];
    message_.angular_velocity.y = data[4];
    message_.angular_velocity.z = data[5];
    
    // Calculate euler angles, convert to quaternion and store in message
    calculateOrientation(data);
    publisher_->publish(message_);
  }
  else
  {
    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
  }
}

void XSensMTi :: timeoutCallback()
{
  if( !rclcpp::ok() )
  {
    rclcpp::shutdown();
    exit(EXIT_SUCCESS);
  }
}

void XSensMTi :: declareParameters()
{
  this->declare_parameter<int>("sample_frequency", 100);
  this->declare_parameter<bool>("calibrated", true);
  this->declare_parameter<bool>("low_latency_serial", true);
  this->declare_parameter<int>("sched_priority", 99);
}

// Taken from https://github.com/hiwad-aziz/ros2_mpu9250_driver/blob/main/src/mpu9250driver.cpp
void XSensMTi :: calculateOrientation( double *data)
{
  // Calculate Euler angles
  double roll, pitch, yaw;
  roll = atan2(message_.linear_acceleration.y, message_.linear_acceleration.z);
  pitch = atan2(-message_.linear_acceleration.y,
                (sqrt(message_.linear_acceleration.y * message_.linear_acceleration.y +
                      message_.linear_acceleration.z * message_.linear_acceleration.z)));
  yaw = atan2(data[7], data[6]);

  // Convert to quaternion
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  message_.orientation.x = cy * cp * sr - sy * sp * cr;
  message_.orientation.y = sy * cp * sr + cy * sp * cr;
  message_.orientation.z = sy * cp * cr - cy * sp * sr;
  message_.orientation.w = cy * cp * cr + sy * sp * sr;  
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XSensMTi>());
  rclcpp::shutdown();
  return 0;
}
