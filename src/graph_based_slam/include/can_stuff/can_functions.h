#ifndef CAN_FUNCTIONS_H
#define CAN_FUNCTIONS_H

#include <math.h>
#include <stdint.h>

#define X   0
#define Y   1
#define Z   2

typedef struct {
  uint8_t AMK_bSystemReady;      //System ready(SBM)
  uint8_t AMK_bError;            //Error
  uint8_t AMK_bWarn;             //Warning
  uint8_t AMK_bQuitDcOn;         //HVactivation acknowledgment
  uint8_t AMK_bDcOn;             //HVactivation level
  uint8_t AMK_bQuitInverterOn;   // RF Controller enable acknowledgment
  uint8_t AMK_bInverterOn;       //Controller enable level
  uint8_t AMK_bDerating;         //Derating (torque limitation active)

  float AMK_ActualVelocity;       //Signed - Unit: rpm - Actual speed value
  float AMK_TorqueCurrent;        //Signed - Raw data for calculating 'actual torque current'Iq See 'Units'on page 61
  unsigned int AMK_Voltage;   //unSigned - Raw data for calculating 'actual dc_bus voltage
  float AMK_Current;  // see PDK
} motorValues1;

typedef struct {
  float AMK_TempMotor;                //Signed - Unit: 0.1 °C - Motor temperature
  float AMK_TempInverter;             //Signed - Unit: 0.1 °C - Cold plate temperature
  float AMK_TempIGBT;                 //Signed - Unit: 0.1 °C - IGBTtemperature
  unsigned int AMK_ErrorInfo;         //Unsigned - Diagnostic number
} motorValues2;

extern const int AMK_VAL_1_IDS[4];
extern const int AMK_VAL_2_IDS[4];

extern motorValues1 motorVal1[4];   //  0 --> FL, 1 --> FR, 2 --> RL, 3 --> RR
extern motorValues2 motorVal2[4];  

extern float battery_pack_tension;

extern float max_bms_voltage;
extern float min_bms_voltage;
extern float mean_bms_voltage;
extern float max_bms_temp;
extern float min_bms_temp;
extern float mean_bms_temp;
extern float max_temp_nslave;

extern float temperatures[5];
extern uint32_t suspensions[4];

extern float accelerations[3];
extern float omegas[3];

extern float bms_lv[8];

extern float lem_current;

extern int throttle;
extern int brake;
extern int steering;
extern int actualVelRPM;

void total_tension_parser(uint8_t* payload );

//BMS_LV
void bms_lv_parser(uint8_t* payload, int id);
float convert_temp_lv(float cell_volt);

//IMU
void imu_parser(uint8_t imu_values[], int id);
float uint32_to_float(uint32_t u);

//SMU
void smu_parser(uint8_t smu_values[], int id);
float ConvertTempToKelvin(int adc_read);

//BMS Voltage
void bms_voltage_parser(uint8_t bms_values[]);
                        
void bms_temp_parser(uint8_t bms_values[]);
                     
float convertBMSvoltage(uint16_t voltage);
float convertBMStemp(uint16_t temp);

//LEM
void lem_parser(unsigned char lem_values[]);
int getMotorIndex(int id);
uint8_t getBit(char c, int bitNumber);

//AMK
void amk_parser(uint8_t* RXB_AmkVal_Data, int id);

void read_AMK_Values1(uint8_t canMsg[], int indexMotor);
void read_AMK_Values2(uint8_t canMsg[], int indexMotor);
int16_t unsigned_to_signed(uint16_t value);

//General
void general_info_parser(uint8_t* payload);

void convert8_to_16(uint16_t destination[], uint8_t input[], int input_length);

#endif