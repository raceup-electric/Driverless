#include <memory>

#include "../eufs_sim/eufs_plugins/gazebo_cone_plugins/include/gazebo_cone_plugins/rclcpp/rclcpp.hpp"
#include "../eufs_msgs/msg/ConeArrayWithCovariance.msg"
#include "../rclcpp/rclcpp.hpp"
#include "../eufs_msgs/msg/ConeWithCovariance.msg"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(): Node("minimal_subscriber"){
    subscription_ = this->create_subscription<eufs_msgs::msg::ConeArrayWithCovariance>(
      "/ground_trouth/cones", 100, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const tutorial_interfaces::msg::Num::SharedPtr msg) const{
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->blu_cones[0]->point->x);
  }
  rclcpp::Subscription<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr subscription_;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}