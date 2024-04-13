// C++ standard headers
#include <iostream>
#include <memory>
#include <chrono>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>

// Project headers
#include <kvaser_reader_driver/msg/car_info.hpp>
#include <graph_based_slam/msg/wheel_speeds.hpp>
#include <eufs_msgs/msg/car_state.hpp>


/**
 * NB: this class is useful only to use real data
 * 
 * Read velocity and steering angle of the car
 * Integrate it using the kinematic bicycle model
 * Publish the obtained pose in a topic, to be used instead of /odometry_integration/car_state in GraphSLAM
 * Use the graph as usual, with the estimated odometry
 * 
 * (TODO: see paper notes for more details)
 */

using std::placeholders::_1;

rclcpp::Node::SharedPtr node_ptr;
eufs_msgs::msg::CarState _state;
builtin_interfaces::msg::Time _last_sim_time;
// _last_sim_time.stamp.sec = 0;
// _last_sim_time.stamp.nanosec = 0;
const double _update_rate= 1000.0;

class OdomEstimation : public rclcpp::Node
{

public:

    //variables
    
    double dt;
    float PI = 3.14159265;
    float wheel_circumference = 2 * PI * 0.2286; //radius of wheel for raceup sg05
    builtin_interfaces::msg::Time curTime;
    double yaw = 0.0;
    float r = 0.2286; // wheel radius raceup sg05
    float l = 0.8; // diatance between rear wheels raceup sg05

    // constructor
    OdomEstimation(const rclcpp::NodeOptions &options) : Node("odom_estimation", options)
    {
        // eufs_msgs::msg::CarState::SharedPtr --> type  of data  used by the simulator odometry message
        // read velocity and steering angle from the car_info topic, containing data parsed from can readings (launch kvaser_parser together with the bag to get this topic)
        // car_info_sub = this->create_subscription<kvaser_reader_driver::msg::CarInfo>("/car_info", 10, std::bind(&OdomEstimation::integrateOdom, this, _1));
        car_info_sub = this->create_subscription<kvaser_reader_driver::msg::CarInfo>("/car_info", 10, std::bind(&OdomEstimation::integrateOdom, this, _1));
        car_state_pub = this->create_publisher<eufs_msgs::msg::CarState>("/car_real_state", 1); //--> publish a odometry msg in same/similar format of the one of the simulator
    }

    virtual ~OdomEstimation(){};

private:

    rclcpp::Subscription<kvaser_reader_driver::msg::CarInfo>::SharedPtr car_info_sub;
    rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr car_state_pub;

    /*
     * Read all the incoming messages and republish them as custom ROS2 messages.
     */

    std::vector<double> ToQuaternion(std::vector<double> &euler) {
    // Abbreviations for the various angular functions
    double cy = cos(euler[0] * 0.5);
    double sy = sin(euler[0] * 0.5);
    double cp = cos(euler[1] * 0.5);
    double sp = sin(euler[1] * 0.5);
    double cr = cos(euler[2] * 0.5);
    double sr = sin(euler[2] * 0.5);

    std::vector<double> q;
    q.reserve(4);
    q[0] = cy * cp * sr - sy * sp * cr;  // x
    q[1] = sy * cp * sr + cy * sp * cr;  // y
    q[2] = sy * cp * cr - cy * sp * sr;  // z
    q[3] = cy * cp * cr + sy * sp * sr;  // w

    return q;
    }

    // calculate each wheel speed from it's rpm
    graph_based_slam::msg::WheelSpeeds getWheelSpeeds(kvaser_reader_driver::msg::CarInfo car_info_msg) {

        graph_based_slam::msg::WheelSpeeds wheel_speeds;

        // wheel_speeds.steering = input.delta;
        wheel_speeds.header = car_info_msg.header;
        wheel_speeds.fl = 999;
        wheel_speeds.fr = 999;

        // Calculate Wheel speeds angular velocity for raceup electric sg05, considering it only as differential drive
        wheel_speeds.rl = 3.5*(2 * PI)*((car_info_msg.rl_actual_velocity  / 60) / 14.44); //devided by the gear ration
        wheel_speeds.rr = 3.5*(2 * PI)*(car_info_msg.rr_actual_velocity  / 60 / 14.44); //devided by the gear ration

        return wheel_speeds;
    }

    void resetCarState(){

        // car_state.header.stamp.sec = wheel_speeds.sec;
        // car_state.header.stamp.nanosec = wheel_speeds.nsec;
        // car_state.header.frame_id = _reference_frame;
        _state.child_frame_id = "base_footprint";

        _state.pose.pose.position.x = 0.0;
        _state.pose.pose.position.y = 0.0;
        _state.pose.pose.position.z = 0.0;

        std::vector<double> orientation = {0.0, 0.0, 0.0};

        orientation = ToQuaternion(orientation);

        _state.pose.pose.orientation.x = orientation[0];
        _state.pose.pose.orientation.y = orientation[1];
        _state.pose.pose.orientation.z = orientation[2];
        _state.pose.pose.orientation.w = orientation[3];

        _state.twist.twist.linear.x = 0.0;
        _state.twist.twist.linear.y = 0.0;
        _state.twist.twist.linear.z = 0.0;

        _state.twist.twist.angular.x = 0.0;
        _state.twist.twist.angular.y = 0.0;
        _state.twist.twist.angular.z = 0.0;

        _state.linear_acceleration.x = 0.0;
        _state.linear_acceleration.y = 0.0;
        _state.linear_acceleration.z = 0.0;

        _state.slip_angle = 0.0;

        _state.state_of_charge = 999;


    }

    void updateCarState(graph_based_slam::msg::WheelSpeeds wheel_speeds, const double dt) {

        _state.header.stamp.sec = wheel_speeds.header.stamp.sec;
        _state.header.stamp.nanosec = wheel_speeds.header.stamp.nanosec;
        // _state.header.frame_id = _reference_frame;
        _state.child_frame_id = "base_footprint";

        float w = r/l * (wheel_speeds.rr - wheel_speeds.rl); // overal angular velocity of the car for sg05
        float v = r/2 * (wheel_speeds.rr + wheel_speeds.rl); // overal linear velocity of the car for sg05

        _state.pose.pose.position.x = v * cos(yaw) * dt - _state.pose.pose.position.x; 
        _state.pose.pose.position.y = v * sin(yaw) * dt - _state.pose.pose.position.y;  //TODO: add icc


        std::vector<double> orientation = {yaw + (w * dt), 0.0, 0.0};
        yaw = w * dt; 
        std::cout<<"yaw: "<<yaw << std::endl;
        orientation = ToQuaternion(orientation);

        _state.pose.pose.orientation.x = orientation[0];
        _state.pose.pose.orientation.y = orientation[1];
        _state.pose.pose.orientation.z = orientation[2];
        _state.pose.pose.orientation.w = orientation[3];

        _state.twist.twist.linear.x = (cos(w) - sin(w)) * r;
        _state.twist.twist.linear.y = (sin(w) + cos(w)) * r;


        _state.twist.twist.angular.x = (cos(w) - sin(w));
        _state.twist.twist.angular.y = (sin(w) + cos(w));



        // _state.slip_angle = _vehicle->getSlipAngle(_state, _act_input, true);

        _state.state_of_charge = 999;    

    }

    void integrateOdom(const kvaser_reader_driver::msg::CarInfo::SharedPtr car_info_msg)
    {
        // use differential drive model to integrate velocity and get the pose of the vehicle
        graph_based_slam::msg::WheelSpeeds wheel_speeds = getWheelSpeeds(*car_info_msg);
        // wheel_speeds_pub->publish(wheel_speeds);

        //update state of the car        
        curTime.sec = wheel_speeds.header.stamp.sec;
        curTime.nanosec = wheel_speeds.header.stamp.nanosec;
        dt = (double)((curTime.sec - _last_sim_time.sec) + (curTime.nanosec - _last_sim_time.nanosec)/1000000000);
        if (dt < (1 / _update_rate)) {
            return;
        }

        _last_sim_time = curTime;
        updateCarState(wheel_speeds, dt);
        car_state_pub->publish(_state);


        //publish wheel speeds
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    node_ptr = std::make_shared<OdomEstimation>(options);
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
