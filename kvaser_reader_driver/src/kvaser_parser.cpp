// Standard headers
#include <stdint.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <sys/mman.h>

// ROS2 headers
#include <rclcpp/rclcpp.hpp>

// Project headers
#include <kvaser_reader_driver/msg/can_data.hpp>
#include <kvaser_reader_driver/msg/car_info.hpp>
#include <kvaser_reader_driver/msg/amk_velocity.hpp>

// CANLIB headers
#include <canlib.h>
#include <kvaser_reader_driver/can_functions.h>
#include <kvaser_reader_driver/IDS.h>
#include <can_msgs/msg/frame.h>

#define TWO_PI 6.283185307179586476925286766559
#define RPM_TO_KMH (TWO_PI * 0.00095)

#define INT 1
#define FLOAT 2
#define BOOL 3

const int AMK_VAL_1_IDS[4] = {0x283, 0x284, 0x287, 0x288};
const int AMK_VAL_2_IDS[4] = {0x285, 0x286, 0x289, 0x28A};

uint64_t time_k = 0;

// Battery_pack_tension
float battery_pack_tension;
// BMS_LV
float bms_lv[8]; // first 6 are tension last two are temps
// IMU
float accelerations[3]; // g
float omegas[3];        // rad/s
// SMU
float temperatures[5];
uint32_t suspensions[4];
// BMS
float max_bms_voltage;
float min_bms_voltage;
float mean_bms_voltage;
float max_bms_temp;
float min_bms_temp;
float mean_bms_temp;
float max_temp_nslave;
// LEM
float lem_current;
// AMK
motorValues1 motorVal1[4]; //  0 --> FL, 1 --> FR, 2 --> RL, 3 --> RR
motorValues2 motorVal2[4];
// GENERAL
int throttle; // % 0-100
int brake;    // % 0-100
int16_t steering; // steering angle in degrees
int actualVelRPM;

using std::placeholders::_1;

rclcpp::Node::SharedPtr node_ptr;

class KvaserParser : public rclcpp::Node
{

public:
    // constructor
    KvaserParser(const rclcpp::NodeOptions &options) : Node("kvaser_reader", options)
    {

        can_sub = this->create_subscription<kvaser_reader_driver::msg::CanData>("/can_data_out", 10, std::bind(&KvaserParser::parseCanMsg, this, _1));
        car_info_pub = this->create_publisher<kvaser_reader_driver::msg::CarInfo>("/car_info", 1);
        amk_velocity_pub = this->create_publisher<kvaser_reader_driver::msg::AmkVelocity>("/amk_velocity", 1);
    }

    virtual ~KvaserParser(){};

private:
    rclcpp::Subscription<kvaser_reader_driver::msg::CanData>::SharedPtr can_sub;
    rclcpp::Publisher<kvaser_reader_driver::msg::CarInfo>::SharedPtr car_info_pub;
    rclcpp::Publisher<kvaser_reader_driver::msg::AmkVelocity>::SharedPtr amk_velocity_pub;
    kvaser_reader_driver::msg::CarInfo car_info_msg;
    int prev_nanosec = 0;
    /**
     * Read all the incoming messages and republish them as custom ROS2 messages.
     */
    void parseCanMsg(const kvaser_reader_driver::msg::CanData::SharedPtr can_msg)
    {
        uint8_t msg_body_int[8];
        //uint8_t msg_body_int[8];
        // parse messages according to their ID and republish to single topics
        switch (can_msg->id)
        {
        // IMU
        // case 0x60:
        // case 0x61:
        // case 0x62:
        //     uint8_t msg_body_int[8];
        //     for (int i = 0; i < can_msg->msg_body.size(); i++)
        //     {
        //         msg_body_int[i] = (uint8_t)can_msg->msg_body[i];
        //     }
        //     imu_parser(msg_body_int, (int)can_msg->id);
        //     printf("acc_x %d %d %.3f\r\n", ACC_X_ID, FLOAT, accelerations[0]);
        //     printf("acc_y %d %d %.3f\r\n", ACC_Y_ID, FLOAT, accelerations[1]);
        //     printf("acc_z %d %d %.3f\r\n", ACC_Z_ID, FLOAT, accelerations[2]);
        //     printf("omega_x %d %d %.3f\r\n", OMG_X_ID, FLOAT, omegas[0]);
        //     printf("omega_y %d %d %.3f\r\n", OMG_Y_ID, FLOAT, omegas[1]);
        //     printf("omega_z %d %d %.3f\r\n", OMG_Z_ID, FLOAT, omegas[2]);

        //     break;

        // Car general info
        case 0x130:
        {
            auto now_time = this->get_clock()->now();
            for (int i = 0; i < can_msg->msg_body.size(); i++)
            {
                msg_body_int[i] = static_cast<uint8_t>(can_msg->msg_body[i]);
            }
            general_info_parser(msg_body_int);
            
            //kvaser_reader_driver::msg::CarInfo car_info_msg;
            car_info_msg.header.stamp = now_time; // timestamp when data has been produced
            car_info_msg.brake_perc = brake;
            car_info_msg.throttle_perc = throttle;
            car_info_msg.steering_angle_deg = steering;
            car_info_msg.car_velocity_kmh = actualVelRPM * RPM_TO_KMH;
            //car_info_pub->publish(car_info_msg);
        }
            break;

        // FL AMK
        case 0x283:
        {
            auto now_time = this->get_clock()->now();
            kvaser_reader_driver::msg::AmkVelocity amk_velocity_msg;
            for (int i = 0; i < can_msg->msg_body.size(); i++)
            {
                msg_body_int[i] = static_cast<uint8_t>(can_msg->msg_body[i]);
            }
            amk_parser(msg_body_int, 0x283);
            
            // amk_velocity_msg.header = can_msg->header; // timestamp when data has been produced
            // amk_velocity_msg.motor_index = 2; //RL motor
            car_info_msg.header.stamp = now_time;
            car_info_msg.fl_actual_velocity = motorVal1[0].AMK_ActualVelocity;
            //amk_velocity_pub->publish(amk_velocity_msg);

        }
            break;
        // FR AMK
        case 0x284:
        {
            auto now_time = this->get_clock()->now();
            kvaser_reader_driver::msg::AmkVelocity amk_velocity_msg;
            for (int i = 0; i < can_msg->msg_body.size(); i++)
            {
                msg_body_int[i] = static_cast<uint8_t>(can_msg->msg_body[i]);
            }
            amk_parser(msg_body_int, 0x284);
            
            // amk_velocity_msg.header = can_msg->header; // timestamp when data has been produced
            // amk_velocity_msg.motor_index = 2; //RL motor
            car_info_msg.header.stamp = now_time;
            car_info_msg.fr_actual_velocity = motorVal1[1].AMK_ActualVelocity;
            //amk_velocity_pub->publish(amk_velocity_msg);

        }
            break;

        // RL AMK
        case 0x287:
        {
            auto now_time = this->get_clock()->now();
            kvaser_reader_driver::msg::AmkVelocity amk_velocity_msg;
            for (int i = 0; i < can_msg->msg_body.size(); i++)
            {
                msg_body_int[i] = static_cast<uint8_t>(can_msg->msg_body[i]);
            }
            amk_parser(msg_body_int, 0x287);
            
            // amk_velocity_msg.header = can_msg->header; // timestamp when data has been produced
            // amk_velocity_msg.motor_index = 2; //RL motor
            car_info_msg.header.stamp = now_time;
            car_info_msg.rl_actual_velocity = motorVal1[2].AMK_ActualVelocity;
            //amk_velocity_pub->publish(amk_velocity_msg);

        }
            break;
        // RR AMK
        case 0x288:
        {
            auto now_time = this->get_clock()->now();
            kvaser_reader_driver::msg::AmkVelocity amk_velocity_msg;
            for (int i = 0; i < can_msg->msg_body.size(); i++)
            {
                msg_body_int[i] = static_cast<uint8_t>(can_msg->msg_body[i]);
            }
            amk_parser(msg_body_int, 0x288);
            // amk_velocity_msg.header = can_msg->header; // timestamp when data has been produced
            // amk_velocity_msg.motor_index = 3; //RR motor
            car_info_msg.header.stamp = now_time;
            car_info_msg.rr_actual_velocity = motorVal1[3].AMK_ActualVelocity; 
            //amk_velocity_pub->publish(amk_velocity_msg);

        }
            break;
        }
  
        car_info_pub->publish(car_info_msg);

        
        
        
        /**
         * switch (id)
                            {

                                // BMS_LV
                            case 0x55:
                            case 0x56:
                                print_timestamp(ans);
                                bms_lv_parser((uint8_t *)msg, (int)id);

                                printf("%d %d %.3f\r\n", BMS_LV_0_ID, FLOAT, bms_lv[0]);
                                printf("%d %d %.3f\r\n", BMS_LV_1_ID, FLOAT, bms_lv[1]);
                                printf("%d %d %.3f\r\n", BMS_LV_2_ID, FLOAT, bms_lv[2]);
                                printf("%d %d %.3f\r\n", BMS_LV_3_ID, FLOAT, bms_lv[3]);
                                printf("%d %d %.3f\r\n", BMS_LV_4_ID, FLOAT, bms_lv[4]);
                                printf("%d %d %.3f\r\n", BMS_LV_5_ID, FLOAT, bms_lv[5]);
                                printf("%d %d %.3f\r\n", BMS_LV_6_ID, FLOAT, bms_lv[6]);
                                printf("%d %d %.3f\r\n", BMS_LV_7_ID, FLOAT, bms_lv[7]);

                                break;



                                // SMU temp and susp
                            case 0x100:
                            case 0x101:
                                print_timestamp(ans);
                                smu_parser((uint8_t *)msg, (int)id);

                                printf("%d %d %.3f\r\n", TEMP_PRE_RADIATOR_ID, FLOAT, temperatures[0]);
                                printf("%d %d %.3f\r\n", TEMP_PRE_COLDPLATE_ID, FLOAT, temperatures[1]);
                                printf("%d %d %.3f\r\n", TEMP_POST_COLDPLATE_ID, FLOAT, temperatures[2]);
                                printf("%d %d %.3f\r\n", TEMP_PRE_MOTOR_ID, FLOAT, temperatures[3]);
                                printf("%d %d %.3f\r\n", TEMP_POST_MOTOR_ID, FLOAT, temperatures[4]);

                                printf("%d %d %.3f\r\n", SUSP_FL_ID, FLOAT, (float)suspensions[0]);
                                printf("%d %d %.3f\r\n", SUSP_FR_ID, FLOAT, (float)suspensions[1]);
                                printf("%d %d %.3f\r\n", SUSP_RL_ID, FLOAT, (float)suspensions[2]);
                                printf("%d %d %.3f\r\n", SUSP_RR_ID, FLOAT, (float)suspensions[3]);

                                break;

                                // BMS Voltage
                            case 0x110:
                                print_timestamp(ans);
                                bms_voltage_parser((uint8_t *)msg);

                                printf("%d %d %.3f\r\n", MAX_BMS_VOLTAGE_ID, FLOAT, max_bms_voltage);
                                printf("%d %d %.3f\r\n", MIN_BMS_VOLTAGE_ID, FLOAT, min_bms_voltage);
                                printf("%d %d %.3f\r\n", MEAN_BMS_VOLTAGE_ID, FLOAT, mean_bms_voltage);

                                break;
                                // BMS Temp
                            case 0x111:
                                print_timestamp(ans);
                                bms_temp_parser((uint8_t *)msg);

                                printf("%d %d %.3f\r\n", MAX_BMS_TEMP_ID, FLOAT, max_bms_temp);
                                printf("%d %d %.3f\r\n", MIN_BMS_TEMP_ID, FLOAT, min_bms_temp);
                                printf("%d %d %.3f\r\n", MEAN_BMS_TEMP_ID, FLOAT, mean_bms_temp);
                                printf("%d %d %.3f\r\n", MAX_NSLAVE_TEMP_ID, FLOAT, max_temp_nslave);

                                break;

                                // Battery_pack_tension + total power
                            case 0x120:
                                print_timestamp(ans);
                                total_tension_parser((uint8_t *)msg);

                                printf("%d %d %.3f\r\n", CAR_VOLTAGE_ID, FLOAT, battery_pack_tension);

                                printf("%d %d %.3f\r\n", TOTAL_POWER_ID, FLOAT, battery_pack_tension * lem_current);

                                // aggiungerr total power da tabella
                                // togliere "\r\n da tutto
                                break;

                                // Car general info
                            case 0x130:
                                print_timestamp(ans);
                                general_info_parser((uint8_t *)msg);

                                // ID, tipo, msg
                                printf("%d %d %d\r\n", THROTTLE_ID, INT, throttle);
                                printf("%d %d %d\r\n", BRAKE_ID, INT, brake);
                                printf("%d %d %d\r\n", STEERING_ID, INT, steering);
                                printf("%d %d %f\r\n", ACTUALVELKMH_ID, FLOAT, actualVelRPM * RPM_TO_KMH); // TODO: check if RPM or kmh (kmh = RPM * 2 pi * 0.00095)
                                break;

                                // LEM
                            case 0x3C2:
                                print_timestamp(ans);
                                lem_parser(msg);

                                printf("%d %d %.3f\r\n", LEM_ID, FLOAT, lem_current);

                                break;

                                // AMK
                            case 0x283:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %d\r\n", AMK_STATUS_0_ID, INT, motorVal1[0].AMK_bSystemReady);
                                printf("%d %d %.3f\r\n", AMK_ACTUAL_VELOCITY_0_ID, FLOAT, motorVal1[0].AMK_ActualVelocity);
                                printf("%d %d %.3f\r\n", AMK_TORQUE_CURRENT_0_ID, FLOAT, motorVal1[0].AMK_TorqueCurrent);
                                printf("%d %d %u\r\n", AMK_VOLTAGE_0_ID, INT, motorVal1[0].AMK_Voltage);
                                printf("%d %d %.3f\r\n", AMK_CURRENT_0_ID, FLOAT, motorVal1[0].AMK_Current);
                                break;
                            case 0x284:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %d\r\n", AMK_STATUS_1_ID, INT, motorVal1[1].AMK_bSystemReady);
                                printf("%d %d %.3f\r\n", AMK_ACTUAL_VELOCITY_1_ID, FLOAT, motorVal1[1].AMK_ActualVelocity);
                                printf("%d %d %.3f\r\n", AMK_TORQUE_CURRENT_1_ID, FLOAT, motorVal1[1].AMK_TorqueCurrent);
                                printf("%d %d %u\r\n", AMK_VOLTAGE_1_ID, INT, motorVal1[1].AMK_Voltage);
                                printf("%d %d %.3f\r\n", AMK_CURRENT_1_ID, FLOAT, motorVal1[1].AMK_Current);
                                break;
                            case 0x285:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %.3f\r\n", AMK_MOTOR_TEMP_0_ID, FLOAT, motorVal2[0].AMK_TempMotor);
                                printf("%d %d %.3f\r\n", AMK_INVERTER_TEMP_0_ID, FLOAT, motorVal2[0].AMK_TempInverter);
                                printf("%d %d %.3f\r\n", AMK_IGBT_TEMP_0_ID, FLOAT, motorVal2[0].AMK_TempIGBT);
                                break;
                            case 0x286:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %.3f\r\n", AMK_MOTOR_TEMP_1_ID, FLOAT, motorVal2[1].AMK_TempMotor);
                                printf("%d %d %.3f\r\n", AMK_INVERTER_TEMP_1_ID, FLOAT, motorVal2[1].AMK_TempInverter);
                                printf("%d %d %.3f\r\n", AMK_IGBT_TEMP_1_ID, FLOAT, motorVal2[1].AMK_TempIGBT);
                                break;
                            case 0x287:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %d\r\n", AMK_STATUS_2_ID, INT, motorVal1[2].AMK_bSystemReady);
                                printf("%d %d %.3f\r\n", AMK_ACTUAL_VELOCITY_2_ID, FLOAT, motorVal1[2].AMK_ActualVelocity);
                                printf("%d %d %.3f\r\n", AMK_TORQUE_CURRENT_2_ID, FLOAT, motorVal1[2].AMK_TorqueCurrent);
                                printf("%d %d %u\r\n", AMK_VOLTAGE_2_ID, INT, motorVal1[2].AMK_Voltage);
                                printf("%d %d %.3f\r\n", AMK_CURRENT_2_ID, FLOAT, motorVal1[2].AMK_Current);
                                break;
                            case 0x288:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %d\r\n", AMK_STATUS_3_ID, INT, motorVal1[3].AMK_bSystemReady);
                                printf("%d %d %.3f\r\n", AMK_ACTUAL_VELOCITY_3_ID, FLOAT, motorVal1[3].AMK_ActualVelocity);
                                printf("%d %d %.3f\r\n", AMK_TORQUE_CURRENT_3_ID, FLOAT, motorVal1[3].AMK_TorqueCurrent);
                                printf("%d %d %u\r\n", AMK_VOLTAGE_3_ID, INT, motorVal1[3].AMK_Voltage);
                                printf("%d %d %.3f\r\n", AMK_CURRENT_3_ID, FLOAT, motorVal1[3].AMK_Current);
                                break;
                            case 0x289:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %.3f\r\n", AMK_MOTOR_TEMP_2_ID, FLOAT, motorVal2[2].AMK_TempMotor);
                                printf("%d %d %.3f\r\n", AMK_INVERTER_TEMP_2_ID, FLOAT, motorVal2[2].AMK_TempInverter);
                                printf("%d %d %.3f\r\n", AMK_IGBT_TEMP_2_ID, FLOAT, motorVal2[2].AMK_TempIGBT);
                                break;
                            case 0x28A:
                                print_timestamp(ans);
                                amk_parser((uint8_t *)msg, (int)id);
                                printf("%d %d %.3f\r\n", AMK_MOTOR_TEMP_3_ID, FLOAT, motorVal2[3].AMK_TempMotor);
                                printf("%d %d %.3f\r\n", AMK_INVERTER_TEMP_3_ID, FLOAT, motorVal2[3].AMK_TempInverter);
                                printf("%d %d %.3f\r\n", AMK_IGBT_TEMP_3_ID, FLOAT, motorVal2[3].AMK_TempIGBT);
                                break;
                            }
        */
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    node_ptr = std::make_shared<KvaserParser>(options);
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
