
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include<math.h>


#define AD0_VAL   1     // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3
SFEVL53L1X distanceSensor0;
//Uncomment the following line to use the optional shutdown and interrupt pins.
// SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN); // uncomment later

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "03cee772-2e79-410b-8846-ce32c73d3bfd"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

// #define ARRAY_LENGTH 200
const int ARRAY_LENGTH = 2000;
#define ARRAY_LENGTH2 1000

// long interval = 500;
// static long previousMillis = 0;
// unsigned long currentMillis = 0;
// unsigned long startTime = 0;

// float Kp = 0;
// float Ki = 0;
// float Kd = 0;

// float maxSpeed = 255;


// const int ARRAY_LENGTH = 200;
// int time_data[ARRAY_LENGTH];
// double temp_data[ARRAY_LENGTH];
// // float pitch_a_data[ARRAY_LENGTH];
// // float roll_a_data[ARRAY_LENGTH];
// // float pitch_a_lpf[ARRAY_LENGTH];
// // float roll_a_lpf[ARRAY_LENGTH];

// // float pitch_g_data[ARRAY_LENGTH];
// // float roll_g_data[ARRAY_LENGTH];
// // float yaw_g_data[ARRAY_LENGTH];

// double pitch_data[ARRAY_LENGTH];
// double roll_data[ARRAY_LENGTH];
// double yaw_data[ARRAY_LENGTH];

// // float pitch_comp_data[ARRAY_LENGTH];
// // float roll_comp_data[ARRAY_LENGTH];

// int distance_data0[ARRAY_LENGTH];
// int distance_data1[ARRAY_LENGTH];

// float pwm_data[ARRAY_LENGTH];
// int fast_time[ARRAY_LENGTH];

// int i = 0;
// unsigned long last_time = millis();
// double targetYaw = 0;
// double currYaw;
// float dt = 0;
// float error_pid;
// float pwm;
// float error_sum;
// float p_term = 0.001;
// float i_term = 0.0;
// float d_term = 0.0;
// float old_error;

//   //Build the Array
// unsigned long start_time = millis();
// bool PID_STRT;

// //////////// Global Variables ////////////
// EString tx_estring_value;
// float tx_float_value = 0.0;
unsigned long start_time = millis();
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
const int data_array = 128; 
// int time_data[data_array];
int temperature_data[data_array];
float roll_a[ARRAY_LENGTH];
float pitch_a[ARRAY_LENGTH];
float roll_a_acc_lpf[ARRAY_LENGTH];
float pitch_a_acc_lpf[ARRAY_LENGTH];
float roll_a_gyro_lpf[ARRAY_LENGTH2];
float pitch_a_gyro_lpf[ARRAY_LENGTH2];
float complementary_pitch[ARRAY_LENGTH];
float complementary_roll[ARRAY_LENGTH];
int time_array_acc[ARRAY_LENGTH];
unsigned long time_array_gyro[ARRAY_LENGTH2];
// unsigned long last_time[ARRAY_LENGTH2];
// int last_time = 0;
// float roll_g[ARRAY_LENGTH2] = {0.0};
float pitch_g = 0;
// float yaw_g[ARRAY_LENGTH2] = {0.0};
float roll_g = 0;
float yaw_g = 0;
float roll_g_array[ARRAY_LENGTH2];
float pitch_g_array[ARRAY_LENGTH2];
float yaw_g_array[ARRAY_LENGTH2];
float distance[ARRAY_LENGTH];
// float dt = 0;
bool send_IMU_data = false;
bool send_IMU_data2 = false;
int IMU_entries_gathered_acc = 0;
int IMU_entries_gathered_gyro = 0;
bool RECORD_IMU = false;

/////////////LAB 3//////////////
// #define SHUTDOWN_PIN 2
// #define INTERRUPT_PIN 3
// SFEVL53L1X distanceSensor1;
// SFEVL53L1X distanceSensor2(Wire, XSHUT);
unsigned long starting_time;
unsigned long e_time;
float time_data_2tof[ARRAY_LENGTH];
// float distance_data1[ARRAY_LENGTH];
float distance_data2[ARRAY_LENGTH];
void collectIMUData_ACC();
void collectIMUData_GYRO();
void collect2TOFData();

 // #define MotorLeftForward 9 //Kelvin's car
// #define MotorLeftBackward 11
// #define MotorRightForward 13
// #define MotorRightBackward 12

//LAB 4
#define PIN0 0 //A1 IN / B1 IN //right forwards
#define PIN1 1 //A2 IN / B2 IN //right backwards
#define PIN2 2 //A1 IN / B1 IN //left backwards
#define PIN3 3 //A2 IN / B2 IN //left forwards

////LAB 5
float current_pos;
float target_pos;
float err, err_d, integral_err;
float pid_p[ARRAY_LENGTH];
float pid_i[ARRAY_LENGTH];
float pid_d[ARRAY_LENGTH];
float pid_tof[ARRAY_LENGTH];
float pid_time[ARRAY_LENGTH];
float pid_speed[ARRAY_LENGTH];
int pid_data_num;
float dt_time, prev_time, prev_err;
float target_tof;
unsigned long current_time;
int max_speed, speed_control;
unsigned long startTime = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;
float maxSpeed = 255;

int time_data[ARRAY_LENGTH];
double temp_data[ARRAY_LENGTH];
// float pitch_a_data[ARRAY_LENGTH];
// float roll_a_data[ARRAY_LENGTH];
// float pitch_a_lpf[ARRAY_LENGTH];
// float roll_a_lpf[ARRAY_LENGTH];

// float pitch_g_data[ARRAY_LENGTH];
// float roll_g_data[ARRAY_LENGTH];
// float yaw_g_data[ARRAY_LENGTH];

double pitch_data[ARRAY_LENGTH];
double roll_data[ARRAY_LENGTH];
double yaw_data[ARRAY_LENGTH];
double p_term_data[ARRAY_LENGTH];
double i_term_data[ARRAY_LENGTH];
double d_term_data[ARRAY_LENGTH];

// float pitch_comp_data[ARRAY_LENGTH];
// float roll_comp_data[ARRAY_LENGTH];

int distance_data0[ARRAY_LENGTH];
int distance_data1[ARRAY_LENGTH];

float pwm_data[ARRAY_LENGTH];
int fast_time[ARRAY_LENGTH];

int i = 0;
unsigned long last_time = millis();
double target_yaw = -30;
double current_yaw = 0;
float dt = 0;
float error_pid;
float pwm;
float error_sum = 0.0;
float p_term = 0.001;
float i_term = 0.0;
float d_term = 0.0;
float old_error;


////LAB 6
  //Build the Array
// unsigned long start_time = millis();
bool pid_starting;

//////////// BLE UUIDs ////////////

enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    TIME_DATA_LOOP,
    SEND_TIME_DATA,
    GET_TEMP_READINGS,
    CALIBRATION,
    START_PID,
    CHANGE_GAIN,
    COLLECT_DMP_DATA, 
    PID_DMP_TURN, 
    PID_ON,
    PID_OFF, 
    PID_KALMAN,
    SEND_KF_DATA
};

void
handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and 
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        /*
         * Write "PONG" on the GATT characteristic BLE_UUID_TX_STRING
         */
        case PING:
            tx_estring_value.clear();
            tx_estring_value.append("PONG");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * Extract two integers from the command string
         */
        case SEND_TWO_INTS:
            int int_a, int_b;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(int_b);
            if (!success)
                return;

            Serial.print("Two Integers: ");
            Serial.print(int_a);
            Serial.print(", ");
            Serial.println(int_b);
            
            break;
        /*
         * Extract three floats from the command string
         */
        case SEND_THREE_FLOATS:
            /*
             * Your code goes here.
             */
            float flt_a, flt_b, flt_c;
            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(flt_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(flt_b);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(flt_c);
            if (!success)
                return;

            Serial.print("3 floats: ");
            Serial.print(flt_a);
            Serial.print(", ");
            Serial.print(flt_b);
            Serial.print(", ");
            Serial.println(flt_c);

            break;
        /*
         * Add a prefix and postfix to the string value extracted from the command string
         */
        case ECHO:

            char char_arr[MAX_MSG_SIZE];

            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;

            /*
             * Your code goes here.
             */
            Serial.print("Robot says -> ");
            Serial.print(char_arr);
            Serial.println(" :)");
            
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        /*
         * DANCE
         */
        case DANCE:
            Serial.println("Look Ma, I'm Dancin'!");

            break;
        
        /*
         * SET_VEL
         */
        case SET_VEL:

            break;
        
        /* 
         * The default case may not capture all types of invalid commands.
         * It is safer to validate the command string on the central device (in python)
         * before writing to the characteristic.
         */
        case GET_TIME_MILLIS:
          float time_millis;
          time_millis = (float) millis();

          Serial.print("T:");
          Serial.println(time_millis);

          tx_estring_value.clear();
          tx_estring_value.append("T:");
          tx_estring_value.append(time_millis);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());

          break;

          case TIME_DATA_LOOP:
            {
            int count = 0;
            unsigned long startT = millis();
            while (millis() - startT < 5000) {
                
                tx_estring_value.clear();
                tx_estring_value.append("Sample ");
                tx_estring_value.append(count);
                tx_estring_value.append(": ");
                tx_estring_value.append((float) millis());
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                count++;

            }

            Serial.println("Done");

            break;
            }
        case SEND_TIME_DATA:
          {
          int i = 0;
          unsigned long startT = millis();

          //Build the Array
          while ((millis() - startT < 500) && (i < ARRAY_LENGTH)) {
              
              time_data[i] = (int) millis();
              i++;

              if(i == ARRAY_LENGTH-1)
              Serial.println("OoM");
              
          }
          for (int j = 0; j < ARRAY_LENGTH; j++) {

              if(time_data[j] == 0)
              break;

              tx_estring_value.clear();
              tx_estring_value.append("Sample ");
              tx_estring_value.append(j);
              tx_estring_value.append(": ");
              tx_estring_value.append(time_data[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());

          }

          Serial.println("Sent time many times");

          break;
          }
        case GET_TEMP_READINGS:
          {
          int i = 0;
          unsigned long startT = millis();

          //Build the Array
          while ( i < ARRAY_LENGTH) {
              
              time_data[i] = (int) millis();
              temp_data[i] = getTempDegF();
              i++;

              if(i == ARRAY_LENGTH-1)
              Serial.println("OoM");
              
          }
          for (int j = 0; j < ARRAY_LENGTH; j++) {

              if(time_data[j] == 0)
              break;

              tx_estring_value.clear();
              tx_estring_value.append("Sample ");
              tx_estring_value.append(j);
              tx_estring_value.append(": Temp is ");
              tx_estring_value.append(temp_data[j]);
              tx_estring_value.append(" at time ");
              tx_estring_value.append(time_data[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());

          }

          Serial.println("Sent time many times");

          break;
          }

          case CALIBRATION: {
             int PIN0_Output, PIN1_Output, PIN2_Output, PIN3_Output;
             success = robot_cmd.get_next_value(PIN0_Output); //backward right
             if (!success)
                 return;
             success = robot_cmd.get_next_value(PIN1_Output); //forward right
             if (!success)
                 return;
             success = robot_cmd.get_next_value(PIN2_Output); //backward left
             if (!success)
                 return;
             success = robot_cmd.get_next_value(PIN3_Output); //forward left
             if (!success)
                 return;
 
             Serial.print("Pins output: ");
             Serial.print(PIN0_Output);
             Serial.print(", ");
             Serial.println(PIN1_Output);
             Serial.print(", ");
             Serial.print(PIN2_Output);
             Serial.print(", ");
             Serial.println(PIN3_Output);
 
            //  analogWrite(PIN0, 0);
            //  analogWrite(PIN1, 80);
            //  analogWrite(PIN2, 0);
            //  analogWrite(PIN3, 70);
            //  delay(2000);
             analogWrite(PIN0, PIN0_Output);
             analogWrite(PIN1, PIN1_Output);
             analogWrite(PIN2, PIN2_Output);
             analogWrite(PIN3, PIN3_Output);
             delay(2000);
             analogWrite(PIN0, 0);
             analogWrite(PIN1, 0);
             analogWrite(PIN2, 0);
             analogWrite(PIN3, 0);

             break;
          }

          case START_PID:
          {
            int i = 0;
            int k = 0;
            float pitch_g = 0, roll_g = 0, yaw_g = 0, dt =0, pitch_g_accum = 0, roll_g_accum = 0, yaw_g_accum = 0;
            unsigned long last_time = millis();
            unsigned long last_time1 = millis();
            const float alpha = 0.2;
            int distance1 = 0;
            float current_dist;
            float error1;
            float target_dist = 304;
            float pwm;
            float error_sum;
            float p_term;
            float i_term;
            float d_term;
            float old_error;

            distanceSensor0.startRanging(); //Write configuration bytes to initiate measurement
            Serial.println("STARTING PID");
              //Build the Array

              int t_0 = millis();
              // This is used to calculate dtin
              int prev_time = t_0; 

              while (i < ARRAY_LENGTH & millis() - t_0 < 5000) {
                if(distanceSensor0.checkForDataReady())
                {
                  distance1 =  distanceSensor0.getDistance(); //Get the result of the measurement from the sensor
                  distanceSensor0.stopRanging();
                  distanceSensor0.clearInterrupt();
                  distanceSensor0.startRanging();
                  i++;
                }
                // Start of Control Loop Calculations
                distance_data1[i] = distance1;
                current_dist = distance1;
                old_error = error1;
                error1 = current_dist - target_dist; //target_dist - current_dist;
                //PID Control
                dt = millis() - prev_time;
                prev_time = millis();
                time_data[i] = prev_time;
                // error_sum = error_sum + (error1 * dt / 1000);
                if (!isnan(error1) && dt > 0) {
                  error_sum += (error1 * dt / 1000.0);
                }
                if (error_sum>150) {
                  error_sum = 150;
                } else if (error_sum < -150) {
                  error_sum = -150;
                }
                p_term = Kp * error1;
                i_term = Ki * error_sum;
                // d_term = 0;
                d_term = Kd * (error1 - old_error)/dt;
                pwm = p_term + i_term + d_term;

                // Control Signal Saturation                  
                if(pwm > maxSpeed) pwm = maxSpeed;
                if(pwm < -maxSpeed) pwm = -maxSpeed;
                
                pwm_data[i] = pwm;
                if(pwm < 0) { //previous code: pwm > 0
                  analogWrite(PIN0,0); // Kelvin's car: right backward
                  analogWrite(PIN1,pwm + 5); // Kelvin's car: right foward
                  analogWrite(PIN3,pwm*1.1); // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                  analogWrite(PIN2,0); // Kelvin's car: left backward
                }
                else if(pwm > 0) { //previous code: pwm < 0 
                  // pwm = abs(pwm);
                  analogWrite(PIN0,abs(pwm) + 5);
                  analogWrite(PIN1,0);
                  analogWrite(PIN3,0);
                  analogWrite(PIN2,abs(pwm)*1.1); //(PIN2,abs(pwm)*2.0 + 12);
                }
                else{
                  analogWrite(PIN0,0);
                  analogWrite(PIN1,0);
                  analogWrite(PIN3,0);
                  analogWrite(PIN2,0);
                }
              }

              analogWrite(PIN0,0);
              analogWrite(PIN1,0);
              analogWrite(PIN3,0);
              analogWrite(PIN2,0);

              for (int j = 0; j < ARRAY_LENGTH; j++) {
                tx_estring_value.clear();
                tx_estring_value.append(distance_data1[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(pwm_data[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(time_data[j]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                Serial.println(tx_characteristic_string);
              }

              Serial.println("START_PID finished sending data!");
            break;
          }
          
          case PID_DMP_TURN: 
          {
            int i = 0;
            unsigned long last_time = millis();
            //double target_yaw = ;
            //double currrent_yaw;
            float dt = 0;
            float error1;
            float pwm;
            float error_sum;
            float p_term;
            float i_term;
            float d_term;
            float old_error;

              //Build the Array
            unsigned long start_time = millis();
            while(millis() - start_time < 20000)
            {
              // unsigned long wait_time = millis();
              // while (millis() - wait_time < 2000) {
              //   icm_20948_DMP_data_t collecting_data;
              //   myICM.readDMPdataFromFIFO(&collecting_data);
              //   delay(10);
              // }
              if (i < ARRAY_LENGTH) {
                icm_20948_DMP_data_t data;
                myICM.readDMPdataFromFIFO(&data);
                if((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
                //|| (myICM.status != ICM_20948_Stat_FIFONoDataAvail))
                {
                  if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
                  { 

                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                    double qw = q0; 
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;

                    // roll (x-axis rotation)
                    double t0 = +2.0 * (qw * qx + qy * qz);
                    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
                    double roll = atan2(t0, t1) * 180.0 / PI;
                    roll_data[i] = roll;

                    // pitch (y-axis rotation)
                    double t2 = +2.0 * (qw * qy - qx * qz);
                    t2 = t2 > 1.0 ? 1.0 : t2;
                    t2 = t2 < -1.0 ? -1.0 : t2;
                    double pitch = asin(t2) * 180.0 / PI;
                    pitch_data[i] = pitch;

                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (qw * qz + qx * qy);
                    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                    double yaw = atan2(t3, t4) * 180.0 / PI;
                    yaw_data[i] = yaw;
                    time_data[i] = millis();
                    i++;

                    dt = (millis()-last_time);
                    last_time = millis();
                    current_yaw = yaw;
                    old_error = error1;
                    error1 = current_yaw-target_yaw;
                    //PID Control
                    error_sum = error_sum + error1*dt;
                    p_term = Kp * error1;
                    i_term = Ki * error_sum;
                    if (error_sum>150) {
                      error_sum = 150;
                    } else if (error_sum < -150) {
                      error_sum = -150;
                    }
                    d_term = Kd * (error1 - old_error)/dt;
                    pwm = p_term + i_term + d_term;
                    p_term_data[i] = p_term;
                    i_term_data[i] = i_term;
                    d_term_data[i] = d_term;
                    Serial.print("i_term: ");
                    Serial.print(i_term);
                    Serial.print(" p_term: ");
                    Serial.print(p_term);
                    Serial.print(" dt: ");
                    Serial.print(dt);
                    Serial.print(" error1: ");
                    Serial.print(error1);
                    Serial.print(" error_sum: ");
                    Serial.print(error_sum);
                    Serial.print(" Current pwm value:");
                    Serial.print(pwm);
                    Serial.print(" Current yaw: ");
                    Serial.print(current_yaw);
                    Serial.print(" target yaw: ");
                    Serial.println(target_yaw);

                    
                    if(pwm > 0)
                    {
                      if(pwm > maxSpeed)
                        pwm = maxSpeed;
                    }
                    else if(pwm < 0)
                    {
                      if(pwm < -maxSpeed)
                        pwm = -maxSpeed;
                    }
                    pwm_data[i] = pwm;
                    if(pwm < -80) { //previous code: pwm > 0 ---> needs to be clockwise motion bc yaw difference is -
                      // pwm = 80+ (255-80)*pwm;
                      analogWrite(PIN0,abs(pwm)); // Kelvin's car: right backward
                      analogWrite(PIN1,0); // Kelvin's car: right foward
                      analogWrite(PIN3,abs(pwm)); // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                      analogWrite(PIN2,0); // Kelvin's car: left backward
                      // analogWrite(PIN0,0); // Kelvin's car: right backward
                      // analogWrite(PIN1,0); // Kelvin's car: right foward
                      // analogWrite(PIN3,0); // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                      // analogWrite(PIN2,0); // Kelvin's car: left backward
                    }
                    else if(pwm > 80)
                        { //previous code: pwm < 0 
                      // pwm = abs(pwm);
                      // pwm = 80+ (255-80)*abs(pwm);
                      analogWrite(PIN0,0);
                      analogWrite(PIN1,abs(pwm));
                      analogWrite(PIN3,0);
                      analogWrite(PIN2,abs(pwm)); //(PIN2,abs(pwm)*2.0 + 12);
                      // analogWrite(PIN0,0);
                      // analogWrite(PIN1,0);
                      // analogWrite(PIN3,0);
                      // analogWrite(PIN2,0); //(PIN2,abs(pwm)*2.0 + 12);
                    }
                    else{
                      analogWrite(PIN0,0);
                      analogWrite(PIN1,0);
                      analogWrite(PIN3,0);
                      analogWrite(PIN2,0);
                    }
                  }
                }
                if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
                {
                  delay(10);
                }
                  
              }
              else{
                icm_20948_DMP_data_t data;
                myICM.readDMPdataFromFIFO(&data);
                if((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
                {
                  if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
                  {

                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                    double qw = q0; // See issue #145 - thank you @Gord1
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;

                    // roll (x-axis rotation)
                    double t0 = +2.0 * (qw * qx + qy * qz);
                    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
                    double roll = atan2(t0, t1) * 180.0 / PI;
                    // roll_data[i] = roll;

                    // pitch (y-axis rotation)
                    double t2 = +2.0 * (qw * qy - qx * qz);
                    t2 = t2 > 1.0 ? 1.0 : t2;
                    t2 = t2 < -1.0 ? -1.0 : t2;
                    double pitch = asin(t2) * 180.0 / PI;
                    // pitch_data[i] = pitch;

                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (qw * qz + qx * qy);
                    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                    double yaw = atan2(t3, t4) * 180.0 / PI;
                    dt = (millis()-last_time);
                    last_time = millis();
                    current_yaw = yaw;
                    old_error = error1;
                    error1 = current_yaw-target_yaw;
                    //PID Control
                    error_sum = error_sum + error1*dt;
                    p_term = Kp * error1;
                    i_term = Ki * error_sum;
                    d_term = Kd * (error1 - old_error)/dt;
                    pwm = p_term + i_term + d_term;
                    

                    if(pwm > 0)
                    {
                      if(pwm > maxSpeed)
                        pwm = maxSpeed;
                    }
                    else if(pwm < 0)
                    {
                      if(pwm < -maxSpeed)
                        pwm = -maxSpeed;
                    }
                    pwm_data[i] = pwm;
                    if(pwm < -50) { //previous code: pwm > 0 ---> needs to be clockwise motion bc yaw difference is -
                      // pwm = 80 + (255-80)*abs(pwm);
                      analogWrite(PIN0,abs(pwm)); // Kelvin's car: right backward
                      analogWrite(PIN1,0); // Kelvin's car: right foward
                      analogWrite(PIN3,abs(pwm)); // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                      analogWrite(PIN2,0); // Kelvin's car: left backward
                      // analogWrite(PIN0,abs(pwm)); // Kelvin's car: right backward
                      // analogWrite(PIN1,0); // Kelvin's car: right foward
                      // analogWrite(PIN3,abs(pwm)); // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                      // analogWrite(PIN2,0); // Kelvin's car: left backward
                    }
                    else if(pwm > 50) { //previous code: pwm < 0 
                      //pwm = abs(pwm);
                      // pwm = 80 + (255-80)*pwm;
                      analogWrite(PIN0,0);
                      analogWrite(PIN1,abs(pwm));
                      analogWrite(PIN3,0);
                      analogWrite(PIN2,abs(pwm)); //(PIN2,abs(pwm)*2.0 + 12);
                      // analogWrite(PIN0,0);
                      // analogWrite(PIN1,abs(pwm));
                      // analogWrite(PIN3,0);
                      // analogWrite(PIN2,abs(pwm)); //(PIN2,abs(pwm)*2.0 + 12);
                    }
                    else{
                      analogWrite(PIN0,0);
                      analogWrite(PIN1,0);
                      analogWrite(PIN3,0);
                      analogWrite(PIN2,0);
                    }
                  }
                }
            }
            // analogWrite(16,0);
            // analogWrite(15,0);
            // analogWrite(14,0);
            // analogWrite(5,0);
            }
            analogWrite(PIN0,0);
            analogWrite(PIN1,0);
            analogWrite(PIN3,0);
            analogWrite(PIN2,0);
            for (int j = 0; j < i; j++) {
              // if(time_data[j] == 0)
              // break;

              tx_estring_value.clear();
              tx_estring_value.append(pitch_data[j]);
              tx_estring_value.append("|");
              tx_estring_value.append(roll_data[j]);
              tx_estring_value.append("|");
              tx_estring_value.append(yaw_data[j]);
              tx_estring_value.append("|");
              tx_estring_value.append(time_data[j]);
              tx_estring_value.append("|");
              tx_estring_value.append(pwm_data[j]);
              tx_estring_value.append("|");
              tx_estring_value.append(p_term_data[j]);
              tx_estring_value.append("|");
              tx_estring_value.append(i_term_data[j]);
              tx_estring_value.append("|");
              tx_estring_value.append(d_term_data[j]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());

            }
          Serial.println("PID turning data sent!");
          break;
          }

          case PID_ON:
          {
            for (int i = 0; i < ARRAY_LENGTH; i++) {
              pitch_data[i] = 0;
              roll_data[i] = 0;
              yaw_data[i] = 0;
              time_data[i] = 0;
              pwm_data[i] = 0;
            }
            i = 0;
            last_time = millis();
            start_time = millis();
            pid_starting = true;
          break;
          }
          case PID_OFF:
          {
            pid_starting = false;
            analogWrite(PIN0,0);
            analogWrite(PIN1,0);
            analogWrite(PIN3,0);
            analogWrite(PIN2,0);
            for (int i = 0; i < i; i++) {
              tx_estring_value.clear();
              tx_estring_value.append(pitch_data[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(roll_data[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(yaw_data[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(time_data[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(pwm_data[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());

            }
          Serial.println("PID_START_OFF finished sending");
          break;
          }

          case CHANGE_GAIN:
          {
            float new_kp; float new_ki; float new_kd; float new_maxSpeed;
            
            success = robot_cmd.get_next_value(new_kp);
            if(!success)
              return;

            success = robot_cmd.get_next_value(new_ki);
            if(!success)
              return;

            success = robot_cmd.get_next_value(new_kd);
            if(!success)
              return;

            success = robot_cmd.get_next_value(new_maxSpeed);
            if(!success)
              return;

            Kp = new_kp;
            Ki = new_ki;
            Kd = new_kd;
            maxSpeed = new_maxSpeed;
            break;
          }
          case COLLECT_DMP_DATA:
          {
            int i = 0;
            unsigned long last_time = millis();

              //Build the Array
              while ( i < ARRAY_LENGTH) {
                icm_20948_DMP_data_t data;
                myICM.readDMPdataFromFIFO(&data);
                if((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
                {
                  if ((data.header & DMP_header_bitmap_Quat6) > 0) 
                  {
                    // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                    //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

                    // Scale to +/- 1
                    double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                    double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                    double qw = q0; 
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;

                    // roll (x-axis rotation)
                    double t0 = +2.0 * (qw * qx + qy * qz);
                    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
                    double roll = atan2(t0, t1) * 180.0 / PI;
                    roll_data[i] = roll;

                    // pitch (y-axis rotation)
                    double t2 = +2.0 * (qw * qy - qx * qz);
                    t2 = t2 > 1.0 ? 1.0 : t2;
                    t2 = t2 < -1.0 ? -1.0 : t2;
                    double pitch = asin(t2) * 180.0 / PI;
                    pitch_data[i] = pitch;

                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (qw * qz + qx * qy);
                    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                    double yaw = atan2(t3, t4) * 180.0 / PI;
                    yaw_data[i] = yaw;
                    time_data[i] = millis();
                    i++;
                  }
                }
                if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
                {
                  delay(10);
                }
              }
              for (int j = 0; j < ARRAY_LENGTH; j++) {
                tx_estring_value.clear();
                tx_estring_value.append(pitch_data[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(roll_data[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(yaw_data[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(time_data[j]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                Serial.println("Sending the value");

              }

              Serial.println("DMP Data Sent!");
          break;
          }
           case PID_KALMAN: 
           {
             distanceSensor1.setIntermeasurementPeriod(20);
             distanceSensor1.setTimingBudgetInMs(20);
             //PID stuff from lab 5
             int i = 0;
             int k = 0;
             int j = 0;
             unsigned long last_time = millis();
             int distance1 = 0;
             float current_dist;
             float error1;
             float target_dist = 304;
             float pwm;
             float error_sum;
             float p_term;
             float i_term;
             float d_term;
             float old_error;
             float kf_distance;
     
             Ad  = I + Delta_T * A ;
             Bd = Delta_T * B;
             Cd = C;
     
             distanceSensor1.startRanging();  //Write configuration bytes to initiate measurement
             Serial.println("Staring Kalman Filter with PID!");
             //Build the Array
             int t_0 = millis();
             // This is used to calculate dtin
             int prev_time = t_0;
             while (!distanceSensor1.checkForDataReady())
             {
                 delay(1);
             }
             distance1 =  distanceSensor1.getDistance();
             distanceSensor1.clearInterrupt();
             mu = {(float) distance1, 0.0};
             while ((i < ARRAY_LENGTH && j < ARRAY_LENGTH2) && millis() - t_0 < 5000) {
               time_data[i] = millis();
               if (distanceSensor1.checkForDataReady()) {
                 dt = millis() - prev_time;
                 prev_time = millis();
                 //time_data[i] = prev_time;
                 distance1 = distanceSensor1.getDistance();  //Get the result of the measurement from the sensor
                 // distanceSensor1.stopRanging();
                 distanceSensor1.clearInterrupt();
                 //distanceSensor1.startRanging();
                 distance_data1[i] = distance1;
                 i++;
                 //Serial.print("TOF Dis:"); Serial.print(distance1); Serial.println();
                 y = (float) distance1; //y(0,0) = (float) distance1;
                 mu_p = Ad*mu + Bd*u;
                 sigma_p = Ad*sigma*~Ad + sigma_u;
                 sigma_m = Cd*sigma_p*~Cd + sigma_z;
                 kkf_gain = sigma_p*~Cd*Inverse(sigma_m);
                 y_m = y-Cd*mu_p;
                 mu = mu_p + kkf_gain * y_m;
                 sigma = I - kkf_gain*Cd*sigma_p;
                 // mus[i] = mu(0);
                 kf_distance = mu(0,0);
                 kf_distance = (float) kf_distance;
                 if (j < ARRAY_LENGTH2) { 
                   kf_distance_array[j] = kf_distance;
                   kf_time[j] = millis();
                   j++;
                   Serial.println(j);
                   //Serial.print("KF Up:"); Serial.print(kf_distance); Serial.println();
                 }
                 //j++;
                 // Start of Control Loop Calculations
                 current_dist = distance1;
                 distance1 = mu(0,0);
                 old_error = error1;
                 error1 = current_dist - target_dist; 
                 //PID Control
                 error_sum = error_sum + (error1 * dt / 1000);
                 p_term = Kp * error1;
                 i_term = Ki * error_sum;
                 if (dt > 0) {
                   d_term = Kd * (error1 - old_error) / dt;
                 } else {
                   d_term = 0;
                 }
                 d_term = Kd * (error1 - old_error) / dt;
                 pwm = p_term + i_term + d_term;
                 //i++;
               }
               else{
                 mu_p = Ad*mu + Bd*u;
                 sigma_p = Ad*sigma*~Ad + sigma_u;
                 kf_distance = mu_p(0,0);
                 kf_distance = (float) kf_distance;
                 mu = mu_p;
                 sigma = sigma_p;
                 if (j < ARRAY_LENGTH2) { 
                   kf_distance_array[j] = kf_distance;
                   kf_time[j] = millis();
                   j++;
                   Serial.println(j);
                   //Serial.print("KF Pr:"); Serial.print(kf_distance); Serial.println();
                 }
                 dt = millis() - prev_time;
                 prev_time = millis();
                 //time_data[i] = prev_time;
                 //run PID loop to get control input ...
                 current_dist = kf_distance;
                 old_error = error1;
                 error1 = current_dist - target_dist; 
                 //PID Control
                 error_sum = error_sum + (error1 * dt / 1000);
                 p_term = Kp * error1;
                 i_term = Ki * error_sum;
                 if (dt > 0) {
                   d_term = Kd * (error1 - old_error) / dt;
                 } else {
                   d_term = 0;
                 }
                 d_term = Kd * (error1 - old_error) / dt;
                 pwm = p_term + i_term + d_term;
                 //j++;
                 Serial.print("dt value is:");
                 Serial.println(dt);
               }
               // Control Signal Saturation
               if (pwm > maxSpeed) pwm = maxSpeed;
               if (pwm < -maxSpeed) pwm = -maxSpeed;
     
               pwm_data[i] = pwm;
               u = pwm/maxSpeed; //170;
               if (pwm < 0) {                   //previous code: pwm > 0
                 analogWrite(MotorRightBackward, 0);          // Kelvin's car: right backward
                 analogWrite(MotorRightForward, abs(pwm) + 5);    // Kelvin's car: right foward
                 analogWrite(MotorLeftForward, abs(pwm) * 1.8 + 12);  // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                 analogWrite(MotorLeftBackward, 0);          // Kelvin's car: left backward
               } else if (pwm > 0) {            //previous code: pwm < 0
                 // pwm = abs(pwm);
                 analogWrite(MotorRightBackward, abs(pwm) + 5);
                 analogWrite(MotorRightForward, 0);
                 analogWrite(MotorLeftForward, 0);
                 analogWrite(MotorLeftBackward, abs(pwm) * 1.8 + 12);  //(PIN2,abs(pwm)*2.0 + 12);
               } else {
                 analogWrite(MotorRightBackward, 0);
                 analogWrite(MotorRightForward, 0);
                 analogWrite(MotorLeftForward, 0);
                 analogWrite(MotorLeftBackward, 0);
               }
             }
             analogWrite(MotorRightBackward, 0);
             analogWrite(MotorRightForward, 0);
             analogWrite(MotorLeftForward, 0);
             analogWrite(MotorLeftBackward, 0);
     
             // for (int k = 0; k < ARRAY_LENGTH; k++) {
             //   if (time_data[k] == 0) break;
             //   else {
             //   tx_estring_value.clear();
             //   tx_estring_value.append(distance_data1[k]);
             //   tx_estring_value.append("|");
             //   tx_estring_value.append(pwm_data[k]);
             //   tx_estring_value.append("|");
             //   tx_estring_value.append(time_data[k]);
             //   tx_estring_value.append("|");
             //   tx_estring_value.append(kf_distance_array[k]);
             //   tx_estring_value.append("|");
             //   tx_estring_value.append(kf_time[k]);
             //   tx_characteristic_string.writeValue(tx_estring_value.c_str());
             //   Serial.println(tx_characteristic_string);
             // }
             // }
             // Serial.println("START_PID finished sending data!");
             break;
           }
     
           case SEND_KF_DATA: 
           {
               for (int k = 0; k < ARRAY_LENGTH; k++) {
                 if (time_data[k] == 0) break;
                 else {
                 tx_estring_value.clear();
                 tx_estring_value.append(distance_data1[k]);
                 tx_estring_value.append("|");
                 tx_estring_value.append(pwm_data[k]);
                 tx_estring_value.append("|");
                 tx_estring_value.append(time_data[k]);
                 tx_estring_value.append("|");
                 tx_estring_value.append(kf_distance_array[k]);
                 tx_estring_value.append("|");
                 tx_estring_value.append(kf_time[k]);
                 tx_estring_value.append("|");
                 tx_estring_value.append(p_term_data[k]);
                 tx_estring_value.append("|");
                 tx_estring_value.append(i_term_data[k]);
                 tx_estring_value.append("|");
                 tx_estring_value.append(d_term_data[k]);
     
                 tx_characteristic_string.writeValue(tx_estring_value.c_str());
                 Serial.println(tx_characteristic_string);
                 }
               }
               Serial.println("START_PID finished sending data!");
               break;
           }
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;

    }
}

void
setup()
{
    // analogWrite(MotorRightBackward,0); Kelvin's car
    // analogWrite(MotorRightForward,0);
    // analogWrite(MotorLeftForward,0);
    // analogWrite(MotorLeftBackward,0);
    pinMode(PIN0, OUTPUT);
    pinMode(PIN1, OUTPUT);
    pinMode(PIN2, OUTPUT);
    pinMode(PIN3, OUTPUT);

    Serial.begin(115200);
    Serial.print("hi");
    
    Serial.print("HELLO");
    Wire.begin();
    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    /*
     * An example using the EString
     */
    // Clear the contents of the EString before using it
    tx_estring_value.clear();

    // Append the string literal "[->"
    tx_estring_value.append("[->");

    // Append the float value
    tx_estring_value.append(9.0);

    // Append the string literal "<-]"
    tx_estring_value.append("<-]");

    // Write the value to the characteristic
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();
    Wire.begin();
    Wire.setClock(400000);
    bool initialized = false;
    while( !initialized )
    {
      myICM.begin( Wire, AD0_VAL );
      Serial.print( F("Initialization of the sensor returned: ") );
      Serial.println( myICM.statusString() );
      if( myICM.status != ICM_20948_Stat_Ok ){
        Serial.println( "Trying again.." );
        delay(500);
      }else{
        initialized = true;
      }
    }
    bool success = true; // In order to confirm that the DMP configuration was successful
    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    // Enable the DMP Game Rotation Vector sensor
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 4) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
      Serial.println("DMP success!");
    }
    else
    {
      Serial.println(F("Enable DMP failed!"));
      Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
      while (1); // Do nothing more
    }

  //   }
    pinMode(SHUTDOWN_PIN, OUTPUT);

    // Set the address of TOF1 to 0xF0
    digitalWrite(SHUTDOWN_PIN, LOW); // Shut down TOF2
    delay(10);
    distanceSensor0.setI2CAddress(0xF0);
    delay(10);
    digitalWrite(SHUTDOWN_PIN, HIGH); // Restart TOF2
    delay(10);

    if (distanceSensor0.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 0 failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    // Serial.println("Sensor online!"); //uncomment later
    // if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
    // {
    //   Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    //   while (1)
    //     ;
    // }
    Serial.println("Sensor online!");
    Serial.println(distanceSensor0.getI2CAddress());
    // Serial.println(distanceSensor1.getI2CAddress()); //uncomment later
    distanceSensor0.setDistanceModeLong(); //Write configuration bytes to initiate measurement
    // distanceSensor1.setDistanceModeLong(); //Write configuration bytes to initiate measurement //uncomment later
    // distanceSensor0.setTimingBudgetInMs(33);
    // distanceSensor1.setTimingBudgetInMs(33);
// measure periodically. Intermeasurement period must be >/= timing budget.
    distanceSensor0.setIntermeasurementPeriod(40);
    // distanceSensor1.setIntermeasurementPeriod(40); //uncomment later
      //RIGHT MOTOR
    // pinMode(MotorRightForward, OUTPUT);
    // pinMode(MotorRightBackward, OUTPUT);
    // //LEFT MOTOR
    // pinMode(MotorLeftForward, OUTPUT);
    // pinMode(MotorLeftBackward, OUTPUT); //11
    
}

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());
        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }
        Serial.println("Disconnected");
    }
}
