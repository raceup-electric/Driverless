#ifndef IDS_H
#define IDS_H

//AMK
//FL
#define AMK_STATUS_0_ID 0
#define AMK_ACTUAL_VELOCITY_0_ID 1
#define AMK_TORQUE_CURRENT_0_ID 2
#define AMK_VOLTAGE_0_ID 3
#define AMK_CURRENT_0_ID 4
#define AMK_MOTOR_TEMP_0_ID 5
#define AMK_INVERTER_TEMP_0_ID 6
#define AMK_IGBT_TEMP_0_ID 7

//FR
#define AMK_STATUS_1_ID 10
#define AMK_ACTUAL_VELOCITY_1_ID 11
#define AMK_TORQUE_CURRENT_1_ID 12
#define AMK_VOLTAGE_1_ID 13
#define AMK_CURRENT_1_ID 14
#define AMK_MOTOR_TEMP_1_ID 15
#define AMK_INVERTER_TEMP_1_ID 16
#define AMK_IGBT_TEMP_1_ID 17

//RL
#define AMK_STATUS_2_ID 20
#define AMK_ACTUAL_VELOCITY_2_ID 21 
#define AMK_TORQUE_CURRENT_2_ID 22
#define AMK_VOLTAGE_2_ID 23
#define AMK_CURRENT_2_ID 24
#define AMK_MOTOR_TEMP_2_ID 25
#define AMK_INVERTER_TEMP_2_ID 26
#define AMK_IGBT_TEMP_2_ID 27

//RR
#define AMK_STATUS_3_ID 30
#define AMK_ACTUAL_VELOCITY_3_ID 31
#define AMK_TORQUE_CURRENT_3_ID 32
#define AMK_VOLTAGE_3_ID 33
#define AMK_CURRENT_3_ID 34
#define AMK_MOTOR_TEMP_3_ID 35
#define AMK_INVERTER_TEMP_3_ID 36
#define AMK_IGBT_TEMP_3_ID 37

//BMS VOLTAGE
#define MAX_BMS_VOLTAGE_ID    44
#define MIN_BMS_VOLTAGE_ID    45
#define MEAN_BMS_VOLTAGE_ID    46

//BMS TEMP
#define MAX_BMS_TEMP_ID    47
#define MIN_BMS_TEMP_ID    48
#define MEAN_BMS_TEMP_ID    49
#define MAX_NSLAVE_TEMP_ID    50 

//BMS_LV
#define BMS_LV_0_ID 60
#define BMS_LV_1_ID 61
#define BMS_LV_2_ID 62
#define BMS_LV_3_ID 63
#define BMS_LV_4_ID 64
#define BMS_LV_5_ID 65
#define BMS_LV_6_ID 66
#define BMS_LV_7_ID 67

//CAR VOLTAGE (ID 0X120)
#define CAR_VOLTAGE_ID 120

//LEM
#define LEM_ID          110

//TOTAL POWER
#define TOTAL_POWER_ID 130

//CAR GENERAL INFO 
#define THROTTLE_ID     40
#define STEERING_ID     41
#define BRAKE_ID        42
#define ACTUALVELKMH_ID    43

//ACCELERATIONS & OMEGAS
#define ACC_X_ID 70
#define ACC_Y_ID 71
#define ACC_Z_ID 72

#define OMG_X_ID 80
#define OMG_Y_ID 81
#define OMG_Z_ID 82

//SMU SUSP & TEMP
#define SUSP_FL_ID 90
#define SUSP_FR_ID 91
#define SUSP_RL_ID 92
#define SUSP_RR_ID 93

#define TEMP_PRE_RADIATOR_ID 100
#define TEMP_PRE_COLDPLATE_ID 101
#define TEMP_POST_COLDPLATE_ID 102
#define TEMP_PRE_MOTOR_ID 103
#define TEMP_POST_MOTOR_ID 104


#endif