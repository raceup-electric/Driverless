// C++ standard headers
#include <iostream>
#include <memory>
#include <chrono>

// ROS headers
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Project headers
#include <kvaser_reader_driver/msg/car_info.hpp>
#include <graph_based_slam/msg/wheel_speeds.hpp>
#include <eufs_msgs/msg/car_state.hpp>

//TF2 headers
#include <tf2_eigen/tf2_eigen.h>


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


class OdomEstimation : public rclcpp::Node
{

public:

    //variables  
    eufs_msgs::msg::CarState _state;

    // constructor
    OdomEstimation(const rclcpp::NodeOptions &options) : Node("odom_estimation", options)
    {
        // eufs_msgs::msg::CarState::SharedPtr --> type  of data  used by the simulator odometry message
        // read velocity and steering angle from the car_info topic, containing data parsed from can readings (launch kvaser_parser together with the bag to get this topic)
        // car_info_sub = this->create_subscription<kvaser_reader_driver::msg::CarInfo>("/car_info", 10, std::bind(&OdomEstimation::integrateOdom, this, _1));
        car_info_sub = this->create_subscription<kvaser_reader_driver::msg::CarInfo>("/car_info", 10, std::bind(&OdomEstimation::integrateOdom, this, _1));
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 10, std::bind(&OdomEstimation::imu_callback, this, _1));
        car_state_pub = this->create_publisher<eufs_msgs::msg::CarState>("/car_real_state", 1); //--> publish a odometry msg in same/similar format of the one of the simulator
        graph_viz_pub = this->create_publisher<visualization_msgs::msg::Marker>("graph_viz", 1);
        
    }

    virtual ~OdomEstimation(){};

private:

    rclcpp::Subscription<kvaser_reader_driver::msg::CarInfo>::SharedPtr car_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Publisher<eufs_msgs::msg::CarState>::SharedPtr car_state_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_viz_pub;

    //variables
    
    long double dt;
    float PI = 3.14159265;
    float wheel_circumference = 2 * PI * 0.2286; //radius of wheel for raceup sg05
    builtin_interfaces::msg::Time curTime;
    double yaw = 0.0;
    float r = 0.2286; // wheel radius raceup sg05
    float r_l = 1.2; // diatance between rear wheels raceup sg05
    float f_l = 1.23; // diatance between front wheels raceup sg05
    visualization_msgs::msg::Marker marker_vertex;
    int id = 0;
    builtin_interfaces::msg::Time _last_sim_time;
    float w; //angular velocity of the car  
    float v; //linear velocity of the car
    // _last_sim_time.stamp.sec = 0;
    // _last_sim_time.stamp.nanosec = 0;
    const double _update_rate= 0.25; //seconds
    float wheels_angle = 0;
    sensor_msgs::msg::Imu this_imu;

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
        q[0] = cy * cp * sr - sy * sp * cr;  // x//the last element converts steering angle to wheel angle
        q[1] = sy * cp * sr + cy * sp * cr;  // y
        q[2] = sy * cp * cr - cy * sp * sr;  // z
        q[3] = cy * cp * cr + sy * sp * sr;  // w

        return q;
    }

    sensor_msgs::msg::Imu imu_converter(const sensor_msgs::msg::Imu &imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;

        double ida[] = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
        std::vector<double> id(ida, std::end(ida));
        std::vector<double> extRotV = id; // extrinsic rotation
        std::vector<double> extRPYV = id; // extrinsic RPY

        double zea[] = {0.0, 0.0, 0.0};
        std::vector<double> ze(zea, std::end(zea));
        std::vector<double> extTransV = ze; // extrinsic translation
        Eigen::Matrix3d extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        Eigen::Matrix3d extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        Eigen::Vector3d extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        Eigen::Quaterniond extQRPY = Eigen::Quaterniond(extRPY);

        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        this_imu = imu_converter(*imu_msg);
    }

    // calculate each wheel speed from it's rpm
    graph_based_slam::msg::WheelSpeeds getWheelSpeeds(kvaser_reader_driver::msg::CarInfo car_info_msg) {

        graph_based_slam::msg::WheelSpeeds wheel_speeds;
        // wheel_speeds.steering = input.delta;
        wheel_speeds.header = car_info_msg.header;

        // Calculate wheels angular velocity for raceup electric sg05 /devided by the gear ration
        wheel_speeds.rl = (2 * PI)*(car_info_msg.rl_actual_velocity  / 60 / 14.44); 
        wheel_speeds.rr = (2 * PI)*(car_info_msg.rr_actual_velocity  / 60 / 14.44); 
        wheel_speeds.fl = (2 * PI)*(car_info_msg.fr_actual_velocity  / 60 / 14.44);
        wheel_speeds.fr = (2 * PI)*(car_info_msg.fl_actual_velocity  / 60 / 14.44);

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
        _state.linear_acceleration.x = 0.0;
        _state.linear_acceleration.y = 0.0;
        _state.linear_acceleration.z = 0.0;

        _state.slip_angle = 0.0;

        _state.state_of_charge = 999;


    }

    void updateCarState(graph_based_slam::msg::WheelSpeeds wheel_speeds, const double dt, kvaser_reader_driver::msg::CarInfo car_info_msg) {

        _state.header.stamp.sec = wheel_speeds.header.stamp.sec;
        _state.header.stamp.nanosec = wheel_speeds.header.stamp.nanosec;
        // _state.header.frame_id = _reference_frame;
        _state.child_frame_id = "base_footprint";

        // kinematic calculations
        wheels_angle =  (PI / 180 ) * 1163 * 0.01273 * car_info_msg.steering_angle_deg * PI / 180; //steering to wheel angle in radians
        v = car_info_msg.car_velocity_kmh / 3.6; // overal linear velocity of the car for sg05, m/s

        if (wheels_angle == 0)
        {
            w = 0;
        }
        else
            w = v / (1.535/tan(wheels_angle)); // overal angular velocity of the car for sg05, just appriximation. TODO: make a better approximation

        // w = this_imu.angular_velocity.z;
        std::cout << "w: " << w << std::endl;
        yaw =  yaw + w * dt;
        _state.pose.pose.position.x = _state.pose.pose.position.x + v * cos(yaw) * dt;
        _state.pose.pose.position.y = _state.pose.pose.position.y + v * sin(yaw) * dt; 
        std::vector<double> orientation = {yaw, 0.0, 0.0};
         
        std::cout<<"yaw: "<<yaw << std::endl;
        std::cout<<"pose x: "<<_state.pose.pose.position.x << std::endl;
        std::cout<<"pose y: "<<_state.pose.pose.position.y << std::endl;
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
        dt = (long double)((long double)curTime.sec + (long double)curTime.nanosec/1000000000) - (long double)((long double)_last_sim_time.sec + (long double)_last_sim_time.nanosec/1000000000);

        //initialization 
        if (_last_sim_time.sec == 0)
        {
            resetCarState();
        }
        
        if (dt < (_update_rate)) {
            return;
        }

        _last_sim_time = curTime;
        updateCarState(wheel_speeds, dt, *car_info_msg);


        //visualization       
     
        marker_vertex.action = visualization_msgs::msg::Marker::MODIFY;
        marker_vertex.type = visualization_msgs::msg::Marker::SPHERE;
        marker_vertex.scale.x = 1;
        marker_vertex.scale.y = 1;
        marker_vertex.scale.z = 1;
        marker_vertex.lifetime = rclcpp::Duration(0);
        marker_vertex.ns = "vertex";

        marker_vertex.header = wheel_speeds.header;
        marker_vertex.header.frame_id = "map";
        marker_vertex.color.r = 0.0;
        marker_vertex.color.g = 1.0;
        marker_vertex.color.b = 0.0;
        marker_vertex.color.a = 1.0;
        marker_vertex.pose.position.x = _state.pose.pose.position.x;
        marker_vertex.pose.position.y = _state.pose.pose.position.y;
        marker_vertex.pose.position.z = 0;
        marker_vertex.id = id;
        id++;

        car_state_pub->publish(_state);
        graph_viz_pub->publish(marker_vertex);


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