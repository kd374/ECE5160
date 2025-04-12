
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <Wire.h>
#include "ICM_20948.h"         // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "SparkFun_VL53L1X.h"  //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include <math.h>
#include <BasicLinearAlgebra.h>    //Use this library to work with matrices:
using namespace BLA;


#define AD0_VAL 1     // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3
SFEVL53L1X distanceSensor0;
//Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);  // uncomment later

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
//#define ARRAY_LENGTH2 1000
const int ARRAY_LENGTH2 = 2000;


// float Kp = 0;
// float Ki = 0;
// float Kd = 0;

// float maxSpeed = 255;


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
float distance[ARRAY_LENGTH];
// float dt = 0;
bool send_IMU_data = false;
bool send_IMU_data2 = false;
int IMU_entries_gathered_acc = 0;
int IMU_entries_gathered_gyro = 0;
bool RECORD_IMU = false;

/////////////LAB 3//////////////
// SFEVL53L1X distanceSensor1;
// SFEVL53L1X distanceSensor2(Wire, XSHUT);
unsigned long starting_time;
unsigned long e_time;
float time_data_2tof[ARRAY_LENGTH];
float distance_data1[ARRAY_LENGTH];
float distance_data2[ARRAY_LENGTH];
void collectIMUData_ACC();
void collectIMUData_GYRO();
void collect2TOFData();

#define MotorLeftForward 9 //Kelvin's car
#define MotorLeftBackward 11
#define MotorRightForward 13
#define MotorRightBackward 12

//LAB 4
#define PIN0 0  //A1 IN / B1 IN //right forwards
#define PIN1 1  //A2 IN / B2 IN //right backwards
#define PIN2 2  //A1 IN / B1 IN //left backwards
#define PIN3 3  //A2 IN / B2 IN //left forwards

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
double pitch_data[ARRAY_LENGTH];
double roll_data[ARRAY_LENGTH];
double yaw_data[ARRAY_LENGTH];
double p_term_data[ARRAY_LENGTH];
double i_term_data[ARRAY_LENGTH];
double d_term_data[ARRAY_LENGTH];

// float pitch_comp_data[ARRAY_LENGTH];
// float roll_comp_data[ARRAY_LENGTH];

int distance_data0[ARRAY_LENGTH];
int distance_data[ARRAY_LENGTH];
// int distance_data1[ARRAY_LENGTH];

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

/////////////LAB 7////////////
float distance_tof0[ARRAY_LENGTH];
float distance_tof1[ARRAY_LENGTH];
float dt_tof0[ARRAY_LENGTH];
float dt_tof1[ARRAY_LENGTH];
float kf_time[ARRAY_LENGTH2];
float start = millis();
float dt_diff = 0;
int go = 1;
int size;

//INITIAL CODE

int kf_distance_array[ARRAY_LENGTH];

float d = 0.26991;
float t_r = 0.42659;//1000; //0.42659
// float m = 0.050003;

float xdot = 3000.0;
// float d = 1/(xdot/1000.0);
// float t_r = 1250.0;
float m = (-d*t_r)/log(.1);

float sigma_1 = 80;
float sigma_2 = 80;
float sigma_3 = 20;

Matrix<2,2> A = {0, 1,
        0, -1*(d/m)}; 
Matrix<2,1> B = {1, 15*(1/m)};  //0, 1/m
Matrix<1,2> C = {1,0}; 

float Delta_T = 0.01; //10.0; //0.2
Matrix<2,2> I = {1, 0,
                    0, 1};
Matrix<2,2> Ad;
// Ad  = I + Delta_T * A ;
Matrix<2,1> Bd;
// Bd = Delta_T * B;
Matrix<1,2> Cd;
// Cd = C;

 Matrix<2,2> sigma_u = {sigma_1*sigma_1, 0,
                        0, sigma_2*sigma_2};
Matrix<1> sigma_z = {sigma_3*sigma_3};

Matrix<2,1> mu_p;
Matrix<2,2> sigma_p;
Matrix<1> sigma_m;

Matrix<2,1> kkf_gain;

Matrix<1> y_m;

Matrix<1> y;

Matrix<2,1> mu = {1,0}; //{set_distance, 0}
Matrix<2,2> sigma = {400, 0, 
                0, 100};

Matrix<1> u = {1};
float KF_distance[ARRAY_LENGTH];
float u_pwm = 0;

//////NEW CODE

// const float Delta_t = 13.145;

// float d = 1 / 2.52;
// float m = ((-0.658 * 1000 * d) / log(1 - 0.9));  

// Matrix<2,2> A = {
//  0,   1,
//  0,  -d/m
// };

// Matrix<2,1> B = {
//  0,
//  1/m
// };

// Matrix<2,2> I = {
//  1, 0,
//  0, 1
// };

// Matrix<2,2> Ad = I + Delta_t * A;

// // Bd = Delta_t * B
// Matrix<2,1> Bd = Delta_t * B;

// Matrix<1,2> C = {
//  1, 0
// };

// Matrix<2,1> x = {
//  distance0,
//  0
// };

// float sigma_1 = 30;
// float sigma_2 = 30;
// float sigma_3 = 5;

// Matrix<2,2> Sigma_u = {
//  sigma_1 * sigma_1,          0,
//  0,           sigma_2 * sigma_2
// };

// Matrix<1,1> Sigma_z = {
//  sigma_3 * sigma_3
// };

// Matrix<2,1> mu;            
// Matrix<2,2> sigma = { 1, 0,
//                      0, 1 };

// Matrix<1,1> u;

// Matrix<1,1> y ;
// Matrix<1,1> y_pred;
// Matrix<2,1> mu_p;

// void kf(const Matrix<1,1>& u, const Matrix<1,1>& y, Matrix<2,1>& mu, Matrix<2,2>& sigma) {
//    Matrix<2,1> mu_p = Ad * mu + Bd * u;

//    Matrix<2,2> sigma_p = Ad * sigma * (~Ad) + Sigma_u;

//    Matrix<1,1> sigma_m = C * sigma_p * (~C) + Sigma_z;

//    Matrix<1,1> sigma_m_inverse = Inverse(sigma_m);

//    Matrix<2,1> kkf_gain = sigma_p * (~C) * sigma_m_inverse;

//    Matrix<1,1> y_m = y - (C * mu_p);

//    mu = mu_p + kkf_gain * y_m;

//    Matrix<2,2> I = { 1, 0, 0, 1 };
//    sigma = (I - kkf_gain * C) * sigma_p;
// }


//////////// BLE UUIDs ////////////


///////////////LAB 8//////////
float error1[ARRAY_LENGTH];
float error1_0[ARRAY_LENGTH];
float motorInputL[ARRAY_LENGTH];
float motorInputR[ARRAY_LENGTH];
float PID_0[ARRAY_LENGTH];
float speedL, speedR, baseL, baseR;
float setDistance;
int counter;




enum CommandTypes {
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
  // TOF_DATA,
  PID_KALMAN,
  PID_KALMAN_UPDATED, 
  KALMAN_REWRITTEN, 
  KALMAN_NEW, 
  SEND_KF_DATA,
  DRIFT,
  STUNT_GAIN,
  SEND_STUNT_DATA,
  FLIP, 
  LAB8_STUNT,
  LAB8
};

// // Kalman Filter helper function - PREVIOUS ITERATION
void KF(float u_scalar, float y_scalar) {
  Matrix<1,1> u_mat = {u_scalar};
  Matrix<1,1> y_mat = {y_scalar};

  mu_p = Ad * mu + Bd * u_mat;
  sigma_p = Ad * sigma * ~Ad + sigma_u;
  sigma_m = Cd * sigma_p * ~Cd + sigma_z;

  Invert(sigma_m);
  kkf_gain = sigma_p * ~Cd * sigma_m;
  y_m = y_mat - Cd * mu_p;
  mu = mu_p + kkf_gain * y_m;
  sigma = (I - kkf_gain * Cd) * sigma_p;
}

////variables for lab 7////
float setpoint = 0.0;
float curr_value = 0.0;
unsigned long PID_curr_time = 0.0;
float PID_last_time = 0.0;
float prev_e;
float e;
float k_p, k_i, k_d, total_error;
int new_sensor = 0; //initially no sensor measurement, thus 0
float dt_kf = 0.01;
float u_kf = 0;


float PID_control(float setpoint, float curr_value)
{
  //compute dt
  PID_curr_time = micros();
  float dt = (PID_curr_time - PID_last_time)/1000000.; //sampling time
  PID_last_time = PID_curr_time;
  prev_e = e;
  e = curr_value - setpoint;
 //P term
  float P = k_p*e;
 //I term
  total_error = total_error + e*dt;
  float I = k_i*total_error;
  //wind-up protection 
  if (I >= 0.05){
    I = 0.05;
  }
  if (I <= -0.05){
    I = - 0.05;
  }
  //D term (initially set to 0)
  float D = k_d * (e - prev_e)/dt;

  //compute motor command
  float motor_out = P + I + D;

  return motor_out;

}

float drive_motors(int dir, float input) {
  int calib = 1.1;
  //scale motor values
  float cmd = 70 + (255 - 70) * input;
  if (dir > 0) {
    analogWrite(MotorRightBackward, 0);          // Kelvin's car: right backward
    analogWrite(MotorRightForward, cmd);    // Kelvin's car: right foward
    analogWrite(MotorLeftForward, cmd * calib);  // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
    analogWrite(MotorLeftBackward, 0);          // Kelvin's car: left backward
  } else if (dir < 0) {            //previous code: pwm < 0
    // pwm = abs(pwm);
    analogWrite(MotorRightBackward, cmd);
    analogWrite(MotorRightForward, 0);
    analogWrite(MotorLeftForward, 0);
    analogWrite(MotorLeftBackward, cmd*calib);  //(PIN2,abs(pwm)*2.0 + 12);
  } else {
    analogWrite(MotorRightBackward, 0);
    analogWrite(MotorRightForward, 0);
    analogWrite(MotorLeftForward, 0);
    analogWrite(MotorLeftBackward, 0);
  }
}

float kalman_filter(float y, float u_kf)
{
  //discrete time matricies
  BLA::Matrix<2,2> Ad = {1+dt_kf*A(0,0), dt_kf*A(0,1),
                    dt_kf*A(1,0), 1+dt_kf*A(1,1)};
  BLA::Matrix<2,1> Bd = {dt_kf*B(0,0),
                dt_kf*B(1,0)};
  //update step
  BLA::Matrix<2> mu_p = {Ad*mu + Bd*u_kf};
  BLA::Matrix<2,2> sigma_p = {Ad * (sigma*~Ad) + sigma_u};

  //predict step - if we have new sensor measurements
  if (new_sensor == 1){
    BLA::Matrix<1,1> sigma_m = {C*(sigma_p*(~C)) + sigma_z};
    BLA::Matrix<1,1> sigma_m_inv = {sigma_m};
    Invert(sigma_m_inv);
    BLA::Matrix<2,1> kkf_gain = {sigma_p*((~C)*sigma_m_inv)};
    BLA:Matrix<1,1> yM = {y};
    BLA::Matrix<1,1> y_m = {yM - C*mu_p};
    mu = mu_p + kkf_gain*y_m;
    sigma = (I - kkf_gain*C)*sigma_p;
    new_sensor = 0; //no longer have a new sensor measurement (just used it)

    return mu(0,0); //return the update position state estimate
  }
  else{ //just use the update step data
    mu = mu_p;
    sigma = sigma_p;
    return mu(0,0); //return the updated position state estimate
  }
}


void handle_command() {
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
      time_millis = (float)millis();

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
          tx_estring_value.append((float)millis());
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

          time_data[i] = (int)millis();
          i++;

          if (i == ARRAY_LENGTH - 1)
            Serial.println("OoM");
        }
        for (int j = 0; j < ARRAY_LENGTH; j++) {

          if (time_data[j] == 0)
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
        while (i < ARRAY_LENGTH) {

          time_data[i] = (int)millis();
          temp_data[i] = getTempDegF();
          i++;

          if (i == ARRAY_LENGTH - 1)
            Serial.println("Array built");
        }
        for (int j = 0; j < ARRAY_LENGTH; j++) {

          if (time_data[j] == 0)
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

    case CALIBRATION:
      {
        int PIN0_Output, PIN1_Output, PIN2_Output, PIN3_Output;
        success = robot_cmd.get_next_value(PIN0_Output);  //backward right
        if (!success)
          return;
        success = robot_cmd.get_next_value(PIN1_Output);  //forward right
        if (!success)
          return;
        success = robot_cmd.get_next_value(PIN2_Output);  //backward left
        if (!success)
          return;
        success = robot_cmd.get_next_value(PIN3_Output);  //forward left
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
        analogWrite(MotorRightBackward, PIN0_Output);
        analogWrite(MotorRightForward, PIN1_Output);
        analogWrite(MotorLeftBackward, PIN2_Output);
        analogWrite(MotorLeftForward, PIN3_Output);
        delay(2000);
        analogWrite(MotorRightBackward, 0);
        analogWrite(MotorRightForward, 0);
        analogWrite(MotorLeftBackward, 0);
        analogWrite(MotorLeftForward, 0);

        break;
      }

    case START_PID:
      {
        int i = 0;
        int k = 0;
        float pitch_g = 0, roll_g = 0, yaw_g = 0, dt = 0, pitch_g_accum = 0, roll_g_accum = 0, yaw_g_accum = 0;
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
        float time_current;
        float distance_last;
        float distance_before_last;
        float time_last;
        float time_before_last;
        float slope;
        float b, m, x;

        distanceSensor1.startRanging();  //Write configuration bytes to initiate measurement
        Serial.println("STARTING PID");
        //Build the Array

        int t_0 = millis();
        // This is used to calculate dtin
        int prev_time = t_0;

        while ((i < ARRAY_LENGTH) && (millis() - t_0 < 10000)) {
          if (distanceSensor1.checkForDataReady()) {
            distance1 = distanceSensor1.getDistance();  //Get the result of the measurement from the sensor
            distanceSensor1.stopRanging();
            distanceSensor1.clearInterrupt();
            distanceSensor1.startRanging();
            i++;
          // } else if (i > 1) {
          //   time_current = millis();
          //   distance_last = distance_data[i - 1];
          //   distance_before_last = distance_data[i - 2];
          //   time_last = time_data[i - 1];
          //   time_before_last = time_data[i - 2];
          //   slope = (distance_last - distance_before_last) / (time_last - time_before_last);
          //   b = distance_last;
          //   m = slope;
          //   x = time_current - time_last;
          //   distance1 = b + m * x;
          }
          // Start of Control Loop Calculations
          distance_data1[i] = distance1;
          current_dist = distance1;
          old_error = error1;
          dt = millis() - prev_time;
          error1 = target_dist - current_dist; //current_dist - target_dist; 
          prev_time = millis();
          time_data[i] = prev_time;
          error_sum = error_sum + (error1 * dt / 1000);
          // if (!isnan(error1) && dt > 0) {
          //   error_sum += (error1 * dt / 1000.0);
          // }
          if (error_sum > 150) {
            error_sum = 150;
          } else if (error_sum < -150) {
            error_sum = -150;
          }
          p_term = Kp * error1;
          i_term = Ki * error_sum;
          // d_term = 0;
          d_term = Kd * (error1 - old_error) / dt;
          pwm = p_term + i_term + d_term;

          // Control Signal Saturation
          if (pwm > maxSpeed) pwm = maxSpeed;
          if (pwm < -maxSpeed) pwm = -maxSpeed;

          pwm_data[i] = pwm;
          if (pwm > 0) {                   //previous code: pwm > 0
            analogWrite(MotorRightBackward, 0);          // Kelvin's car: right backward
            analogWrite(MotorRightForward, pwm + 5);    // Kelvin's car: right foward
            analogWrite(MotorLeftForward, pwm *2.0 + 12);  // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
            analogWrite(MotorLeftBackward, 0);          // Kelvin's car: left backward
          } else if (pwm < 0) {            //previous code: pwm < 0
            // pwm = abs(pwm);
            analogWrite(MotorRightBackward, abs(pwm) + 5);
            analogWrite(MotorRightForward, 0);
            analogWrite(MotorLeftForward, 0);
            analogWrite(MotorLeftBackward, abs(pwm) *2 + 12);  //(PIN2,abs(pwm)*2.0 + 12);
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
        while (millis() - start_time < 20000) {
          // unsigned long wait_time = millis();
          // while (millis() - wait_time < 2000) {
          //   icm_20948_DMP_data_t collecting_data;
          //   myICM.readDMPdataFromFIFO(&collecting_data);
          //   delay(10);
          // }
          if (i < ARRAY_LENGTH) {
            icm_20948_DMP_data_t data;
            myICM.readDMPdataFromFIFO(&data);
            if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
            //|| (myICM.status != ICM_20948_Stat_FIFONoDataAvail))
            {
              if ((data.header & DMP_header_bitmap_Quat6) > 0)  // We have asked for GRV data so we should receive Quat6
              {

                double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
                double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
                double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30
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

                dt = (millis() - last_time);
                last_time = millis();
                current_yaw = yaw;
                old_error = error1;
                error1 = current_yaw - target_yaw;
                //PID Control
                error_sum = error_sum + error1 * dt;
                p_term = Kp * error1;
                i_term = Ki * error_sum;
                if (error_sum > 150) {
                  error_sum = 150;
                } else if (error_sum < -150) {
                  error_sum = -150;
                }
                d_term = Kd * (error1 - old_error) / dt;
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


                if (pwm > 0) {
                  if (pwm > maxSpeed)
                    pwm = maxSpeed;
                } else if (pwm < 0) {
                  if (pwm < -maxSpeed)
                    pwm = -maxSpeed;
                }
                pwm_data[i] = pwm;
                if (pwm < -80) {  //previous code: pwm > 0 ---> needs to be clockwise motion bc yaw difference is -
                  // pwm = 80+ (255-80)*pwm;
                  analogWrite(MotorRightBackward, abs(pwm));  // Kelvin's car: right backward //my car" PIN0
                  analogWrite(MotorRightForward, 0);         // Kelvin's car: right foward //my car PIN1
                  analogWrite(MotorLeftForward, abs(pwm)*2 + 12);  // Kelvin's car: left forward (PIN3,pwm*2.0 + 12); //my car PIN3
                  analogWrite(MotorLeftBackward, 0);         // Kelvin's car: left backward //my car PIN2
                  // analogWrite(PIN0,0); // Kelvin's car: right backward
                  // analogWrite(PIN1,0); // Kelvin's car: right foward
                  // analogWrite(PIN3,0); // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                  // analogWrite(PIN2,0); // Kelvin's car: left backward
                } else if (pwm > 80) {  //previous code: pwm < 0
                  // pwm = abs(pwm);
                  // pwm = 80+ (255-80)*abs(pwm);
                  analogWrite(MotorRightBackward, 0);
                  analogWrite(MotorRightForward, abs(pwm));
                  analogWrite(MotorLeftForward, 0);
                  analogWrite(MotorLeftBackward, abs(pwm));  //(PIN2,abs(pwm)*2.0 + 12);
                  // analogWrite(PIN0,0);
                  // analogWrite(PIN1,0);
                  // analogWrite(PIN3,0);
                  // analogWrite(PIN2,0); //(PIN2,abs(pwm)*2.0 + 12);
                } else {
                  analogWrite(MotorRightBackward, 0);
                  analogWrite(MotorRightForward, 0);
                  analogWrite(MotorLeftForward, 0);
                  analogWrite(MotorLeftBackward, 0);
                }
              }
            }
            if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)  // If more data is available then we should read it right away - and not delay
            {
              delay(10);
            }

          } else {
            icm_20948_DMP_data_t data;
            myICM.readDMPdataFromFIFO(&data);
            if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
              if ((data.header & DMP_header_bitmap_Quat6) > 0)  // We have asked for GRV data so we should receive Quat6
              {

                double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
                double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
                double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                double qw = q0;  // See issue #145 - thank you @Gord1
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
                dt = (millis() - last_time);
                last_time = millis();
                current_yaw = yaw;
                old_error = error1;
                error1 = current_yaw - target_yaw;
                //PID Control
                error_sum = error_sum + error1 * dt;
                p_term = Kp * error1;
                i_term = Ki * error_sum;
                d_term = Kd * (error1 - old_error) / dt;
                pwm = p_term + i_term + d_term;


                if (pwm > 0) {
                  if (pwm > maxSpeed)
                    pwm = maxSpeed;
                } else if (pwm < 0) {
                  if (pwm < -maxSpeed)
                    pwm = -maxSpeed;
                }
                pwm_data[i] = pwm;
                if (pwm < -50) {  //previous code: pwm > 0 ---> needs to be clockwise motion bc yaw difference is -
                  // pwm = 80 + (255-80)*abs(pwm);
                  analogWrite(MotorRightBackward, abs(pwm));  // Kelvin's car: right backward
                  analogWrite(MotorRightForward, 0);         // Kelvin's car: right foward
                  analogWrite(MotorLeftForward, abs(pwm));  // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                  analogWrite(MotorLeftBackward, 0);         // Kelvin's car: left backward
                  // analogWrite(PIN0,abs(pwm)); // Kelvin's car: right backward
                  // analogWrite(PIN1,0); // Kelvin's car: right foward
                  // analogWrite(PIN3,abs(pwm)); // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
                  // analogWrite(PIN2,0); // Kelvin's car: left backward
                } else if (pwm > 50) {  //previous code: pwm < 0
                  //pwm = abs(pwm);
                  // pwm = 80 + (255-80)*pwm;
                  analogWrite(MotorRightBackward, 0);
                  analogWrite(MotorRightForward, abs(pwm));
                  analogWrite(MotorLeftForward, 0);
                  analogWrite(MotorLeftBackward, abs(pwm));  //(PIN2,abs(pwm)*2.0 + 12);
                  // analogWrite(PIN0,0);
                  // analogWrite(PIN1,abs(pwm));
                  // analogWrite(PIN3,0);
                  // analogWrite(PIN2,abs(pwm)); //(PIN2,abs(pwm)*2.0 + 12);
                } else {
                  analogWrite(MotorRightBackward, 0);
                  analogWrite(MotorRightForward, 0);
                  analogWrite(MotorLeftForward, 0);
                  analogWrite(MotorLeftBackward, 0);
                }
              }
            }
          }
          // analogWrite(16,0);
          // analogWrite(15,0);
          // analogWrite(14,0);
          // analogWrite(5,0);
        }
        analogWrite(MotorRightBackward, 0);
        analogWrite(MotorRightForward, 0);
        analogWrite(MotorLeftForward, 0);
        analogWrite(MotorLeftBackward, 0);
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

    case CHANGE_GAIN:
      {
        float new_kp;
        float new_ki;
        float new_kd;
        float new_maxSpeed;

        success = robot_cmd.get_next_value(new_kp);
        if (!success)
          return;

        success = robot_cmd.get_next_value(new_ki);
        if (!success)
          return;

        success = robot_cmd.get_next_value(new_kd);
        if (!success)
          return;

        success = robot_cmd.get_next_value(new_maxSpeed);
        if (!success)
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
        while (i < ARRAY_LENGTH) {
          icm_20948_DMP_data_t data;
          myICM.readDMPdataFromFIFO(&data);
          if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
            if ((data.header & DMP_header_bitmap_Quat6) > 0) {
              // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
              //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

              // Scale to +/- 1
              double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
              double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
              double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

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
          if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail)  // If more data is available then we should read it right away - and not delay
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
    // case TOF_DATA:
    //   {
    //     distanceSensor0.setIntermeasurementPeriod(20);
    //     distanceSensor0.setTimingBudgetInMs(20);
    //     distanceSensor1.setIntermeasurementPeriod(20);
    //     distanceSensor1.setTimingBudgetInMs(20);
    //     int PIN0_Output, PIN1_Output, PIN2_Output, PIN3_Output;
    //     success = robot_cmd.get_next_value(PIN0_Output);  //backward right
    //     if (!success)
    //       return;
    //     success = robot_cmd.get_next_value(PIN1_Output);  //forward right
    //     if (!success)
    //       return;
    //     success = robot_cmd.get_next_value(PIN2_Output);  //backward left
    //     if (!success)
    //       return;
    //     success = robot_cmd.get_next_value(PIN3_Output);  //forward left
    //     if (!success)
    //       return;

    //     // Serial.print("Pins output: ");
    //     // Serial.print(PIN0_Output);
    //     // Serial.print(", ");
    //     // Serial.println(PIN1_Output);
    //     // Serial.print(", ");
    //     // Serial.print(PIN2_Output);
    //     // Serial.print(", ");
    //     // Serial.println(PIN3_Output);

    //     //  analogWrite(PIN0, 0);
    //     //  analogWrite(PIN1, 80);
    //     //  analogWrite(PIN2, 0);
    //     //  analogWrite(PIN3, 70);
    //     // analogWrite(PIN0, PIN0_Output);
    //     // analogWrite(PIN1, PIN1_Output);
    //     // analogWrite(PIN2, PIN2_Output);
    //     // analogWrite(PIN3, PIN3_Output);
    //     //  delay(2000);
    //     // delay(5000);
    //     // analogWrite(PIN0, 0);
    //     // analogWrite(PIN1, 0);
    //     // analogWrite(PIN2, 0);
    //     // analogWrite(PIN3, 0);

    //     analogWrite(MotorRightBackward, PIN0_Output);
    //     analogWrite(MotorRightForward, PIN1_Output);
    //     analogWrite(MotorLeftBackward, PIN2_Output);
    //     analogWrite(MotorLeftForward, PIN3_Output);

    //     Serial.println("VL53L1X Qwiic Test");
    //     distanceSensor1.setDistanceModeLong();
    //     Serial.println("Sensor 0 is online!");

    //     for (int i = 0; i < ARRAY_LENGTH; i++) {
    //       distanceSensor1.startRanging();
    //       // start_time = millis();
    //       unsigned long timeout = millis();
    //       if (!distanceSensor1.checkForDataReady()) {
    //         if (millis() - timeout > 500) {
    //           Serial.println("Sensor timeout!");
    //           break;
    //         }
    //         distance_tof0[i] = (distanceSensor1.getDistance());
    //         distanceSensor1.clearInterrupt();
    //         // distanceSensor0.stopRanging();
    //         dt_tof0[i] = millis() - starting_time;
    //       }
    //       else {
    //         distance_tof0[i] = (distanceSensor1.getDistance());
    //         dt_tof0[i] = millis() - starting_time;
    //       }
    //       // distance_tof0[i] = (distanceSensor0.getDistance());
    //       // distanceSensor0.clearInterrupt();
    //       // distanceSensor0.stopRanging();
    //       // dt_tof0[i] = millis() - starting_time;
    //     }
    //     analogWrite(MotorRightBackward, 0);
    //     analogWrite(MotorRightForward, 0);
    //     analogWrite(MotorLeftBackward, 0);
    //     analogWrite(MotorLeftForward, 0);

    //     Serial.println("Sending data...");
    //     for (int j = 0; j < ARRAY_LENGTH; j++) {
    //       tx_estring_value.clear();
    //       tx_estring_value.append(dt_tof0[j]);
    //       tx_estring_value.append("|");
    //       tx_estring_value.append(distance_tof0[j]);
    //       tx_characteristic_string.writeValue(tx_estring_value.c_str());
    //       Serial.print("Time of TOF0 is: ");
    //       Serial.print(dt_tof0[j]);
    //       Serial.print("Distance: ");
    //       Serial.println(distance_tof0[j]);
    //     }
    //     Serial.print("Sensor 0 ToF data sent!");
    //     break;
    //   }


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

      case PID_KALMAN_UPDATED:
      {
        //PID stuff from lab 5
        int i = 0;
        int k = 0;
        int j = 0;
        int s = 0;
        unsigned long last_time = millis();
        unsigned long start_time = millis();
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

        //Kalman Filter stuff from lab 7
        // float d = 0.26991;
        // float t_r = 0.42659;
        // float m = 0.050003;

        // Matrix<2,2> A = {0, 1,
        //         0, -5.3977}; 
        // Matrix<2,1> B = {1, 19.9987}; 
        // Matrix<1,2> C = {1,0}; 

        // float Delta_T = 0.2;
        // Matrix<2,2> I = {1, 0,
        //                     0, 1};
        // Matrix<2,2> Ad;
        // Ad  = I + Delta_T * A ;
        // Matrix<2,1> Bd;
        // Bd = Delta_T * B;
        // Matrix<1,2> Cd;
        // Cd = C;

        // float sigma_1 = 70;
        // float sigma_2 = 70;
        // float sigma_3 = 20;

        // Matrix<2,2> sigma_u = {sigma_1*sigma_1, 0,
        //                         0, sigma_2*sigma_2};
        // Matrix<1> sigma_z = {sigma_3*sigma_3};

        // Matrix<2,1> mu_p;
        // Matrix<2,2> sigma_p;
        // Matrix<1> sigma_m;
        // Matrix<2,1> kkf_gain;
        // Matrix<1> y_m;
        // Matrix<1> y;

        // Matrix<2,1> mu = {1,0};
        // Matrix<2,2> sigma = {400, 0, 
        //                 0, 100};
        // Matrix<1> u = {1};
        Serial.println("Start PID!");
        while (!distanceSensor1.checkForDataReady())
        {
          delay(1);
        }
        distance1 =  distanceSensor1.getDistance();
        distanceSensor1.clearInterrupt();
        mu = {(float) distance1, 0.0};
        Serial.print("Starting PID with Kalman Filter!");
        while ( i < ARRAY_LENGTH && millis() - start_time < 20000) {
          if(distanceSensor1.checkForDataReady())
          {
            dt = (millis()-last_time);
            last_time = millis();
            distance1 =  distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
            distance_data1[i] = distance1;
            distanceSensor1.clearInterrupt();
            time_data[i] = last_time;
            y = {(float)distance1};
            mu_p = Ad*mu + Bd*u;
            sigma_p = Ad*sigma*~Ad + sigma_u;
            sigma_m = Cd*sigma_p*~Cd + sigma_z;
            kkf_gain = sigma_p*~Cd*Inverse(sigma_m);
            y_m = y-Cd*mu_p;
            mu = mu_p + kkf_gain*y_m;
            sigma = I - kkf_gain*Cd*sigma_p;
            kf_distance_array[i] = mu(0);
            distance1 = mu(0);
            current_dist = distance1;
            old_error = error1;
            error1 = current_dist-target_dist;
            //Proportional Control
            error_sum = error_sum + error1*dt;
            p_term = Kp * error1;
            i_term = Ki * error_sum;
            d_term = Kd * (error1 - old_error)/dt;
            pwm = p_term + i_term + d_term;
            p_term_data[i] = p_term;
            i_term_data[i] = i_term;
            d_term_data[i] = d_term;

            if (pwm > maxSpeed) pwm = maxSpeed;
            if (pwm < -maxSpeed) pwm = -maxSpeed;
            pwm_data[i] = pwm;

            //u = pwm;
            if (pwm < 0) {                   //previous code: pwm > 0
              analogWrite(MotorRightBackward, 0);          // Kelvin's car: right backward
              analogWrite(MotorRightForward, abs(pwm) + 5);    // Kelvin's car: right foward
              analogWrite(MotorLeftForward, abs(pwm) * 1.1);  // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
              analogWrite(MotorLeftBackward, 0);          // Kelvin's car: left backward
            } else if (pwm > 0) {            //previous code: pwm < 0
              // pwm = abs(pwm);
              analogWrite(MotorRightBackward, abs(pwm) + 5);
              analogWrite(MotorRightForward, 0);
              analogWrite(MotorLeftForward, 0);
              analogWrite(MotorLeftBackward, abs(pwm) * 1.1);  //(PIN2,abs(pwm)*2.0 + 12);
            } else {
              analogWrite(MotorRightBackward, 0);
              analogWrite(MotorRightForward, 0);
              analogWrite(MotorLeftForward, 0);
              analogWrite(MotorLeftBackward, 0);
            }
            i++;
          }
        }
        analogWrite(MotorRightBackward, 0);
        analogWrite(MotorRightForward, 0);
        analogWrite(MotorLeftForward, 0);
        analogWrite(MotorLeftBackward, 0);

        for (int s = 0; s < i; s++) {
          if (time_data[s] == 0) break;
          else {
          tx_estring_value.clear();
          tx_estring_value.append(distance_data1[s]);
          tx_estring_value.append("|");
          tx_estring_value.append(pwm_data[s]);
          tx_estring_value.append("|");
          tx_estring_value.append(time_data[s]);
          tx_estring_value.append("|");
          tx_estring_value.append(kf_distance_array[s]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          Serial.println(tx_characteristic_string);
          }
        }

        Serial.println("START_PID finished sending data!");
        break;
      }

      case KALMAN_REWRITTEN:
      {
        int i = 0, j = 0, k = 0;
        float current_dist, error1 = 0, old_error = 0;
        float pwm = 0, error_sum = 0;
        float p_term, i_term, d_term;
        float kf_distance;
        int distance1 = 0;
        unsigned long last_time = millis();
        unsigned long start_time = millis();
        float target_dist = 304;

        // Kalman Init
        Ad = I + Delta_T * A;
        Bd = Delta_T * B;
        Cd = C;

        // Init first Kalman state
        distanceSensor1.startRanging();
        while (!distanceSensor1.checkForDataReady()) delay(1);
        distance1 = distanceSensor1.getDistance();
        distanceSensor1.clearInterrupt();
        mu = { (float)distance1, 0.0 };

        Serial.println("Starting Kalman Filter with PID!");
        int t_0 = millis();
        int prev_time = t_0;
        unsigned long dt;

        while ((i < ARRAY_LENGTH && j < ARRAY_LENGTH2) && (millis() - t_0 < 5000)) {
          if (distanceSensor1.checkForDataReady()) {
            dt = millis() - prev_time;
            prev_time = millis();

            distance1 = distanceSensor1.getDistance();
            distanceSensor1.stopRanging();
            distanceSensor1.clearInterrupt();
            distanceSensor1.startRanging();
            time_data[i] = prev_time;
            distance_data1[i] = distance1;
            // --- Run Kalman Filter ---
            KF(pwm/70, (float)distance1);

            kf_distance = mu(0,0);
            kf_distance_array[i] = kf_distance;
            // --- PID Control ---
            current_dist = distance1;
            old_error = error1;
            error1 = current_dist - target_dist;
            error_sum += error1 * (dt / 1000.0);
            p_term = Kp * error1;
            i_term = Ki * error_sum;
            d_term = (dt > 0) ? Kd * (error1 - old_error) / dt : 0;

            pwm = p_term + i_term + d_term;
            pwm_data[i] = pwm;

            // Apply PWM to motors
            if (pwm < 0) {
              analogWrite(MotorRightBackward, 0);
              analogWrite(MotorRightForward, pwm + 5);
              analogWrite(MotorLeftForward, pwm * 1.1);
              analogWrite(MotorLeftBackward, 0);
            } else if (pwm > 0) {
              analogWrite(MotorRightBackward, abs(pwm) + 5);
              analogWrite(MotorRightForward, 0);
              analogWrite(MotorLeftForward, 0);
              analogWrite(MotorLeftBackward, abs(pwm) * 1.1);
            } else {
              analogWrite(MotorRightBackward, 0);
              analogWrite(MotorRightForward, 0);
              analogWrite(MotorLeftForward, 0);
              analogWrite(MotorLeftBackward, 0);
            }
          }
          else {
            // No new measurement, just predict
            dt = millis() - prev_time;
            prev_time = millis();

            KF(pwm/70, mu(0,0)); // Use previous prediction as dummy "measurement"
            kf_distance = mu(0,0);
            kf_distance_array[i] = kf_distance;
            time_data[i] = prev_time;
            // PID
            current_dist = distance1;
            old_error = error1;
            error1 = current_dist - target_dist;

            error_sum += error1 * (dt / 1000.0);
            p_term = Kp * error1;
            i_term = Ki * error_sum;
            d_term = (dt > 0) ? Kd * (error1 - old_error) / dt : 0;

            pwm = p_term + i_term + d_term;
            pwm_data[i] = pwm;

            if (pwm < 0) {
              analogWrite(MotorRightBackward, 0);
              analogWrite(MotorRightForward, pwm + 5);
              analogWrite(MotorLeftForward, pwm * 1.1);
              analogWrite(MotorLeftBackward, 0);
            } else if (pwm > 0) {
              analogWrite(MotorRightBackward, abs(pwm) + 5);
              analogWrite(MotorRightForward, 0);
              analogWrite(MotorLeftForward, 0);
              analogWrite(MotorLeftBackward, abs(pwm) * 1.1);
            } else {
              analogWrite(MotorRightBackward, 0);
              analogWrite(MotorRightForward, 0);
              analogWrite(MotorLeftForward, 0);
              analogWrite(MotorLeftBackward, 0);
            }
          }
          i++;
          // Clamp PWM
          if (pwm > maxSpeed) pwm = maxSpeed;
          if (pwm < -maxSpeed) pwm = -maxSpeed;
        }

        // Stop Motors
        analogWrite(MotorRightBackward, 0);
        analogWrite(MotorRightForward, 0);
        analogWrite(MotorLeftBackward, 0);
        analogWrite(MotorLeftForward, 0);

        // Transmit results
        for (int k = 0; k < i; k++) {
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
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          Serial.println(tx_characteristic_string);
        }
        
        }
        Serial.println("PID_KALMAN data sent!");
        break;
      }

      // case KALMAN_NEW:
      // {
      // float Kp, Ki, Kd, PWM_MIN, PWM_MAX;
      // success = robot_cmd.get_next_value(Kp);
      // if (!success) return;

      // success = robot_cmd.get_next_value(Ki);
      // if (!success) return;

      // success = robot_cmd.get_next_value(Kd);
      // if (!success) return;

      // success = robot_cmd.get_next_value(PWM_MIN);
      // if (!success) return;

      // success = robot_cmd.get_next_value(PWM_MAX);
      // if (!success) return;

      // start_time_1 = millis();
      // sensor2.startRanging();

      // u(0,0) = 1;

      // while (millis() - start_time_1 < 5000) {  // Run for 5 seconds
      //     unsigned long now = millis();
      //     float currentDistance = 0.0;
      //     bool measurementAvailable = sensor2.checkForDataReady();
      //     if (measurementAvailable) {
      //         float measuredValue = sensor2.getDistance();

      //         // If the filter hasn't been initialized, use the first measurement.
      //         if (!kalman_initialized) {
      //             mu(0,0) = measuredValue;  // Initialize position
      //             mu(1,0) = 0;              // Assume initial velocity = 0
      //             kalman_initialized = true;
      //         } else {
      //             // With filter already initialized, perform a full Kalman filter update.
      //             y(0,0) = measuredValue;
      //             u(0,0) = abs((float)pwm_vals[index - 1])/175;
      //             kf(u, y, mu, sigma);
      //         }
      //         measured_distances[measure_index] = measuredValue;
      //         measured_times[measure_index] = now;
      //         sensor2.clearInterrupt();
      //         measure_index++;
      //     }
      //     else {
      //         // No new measurement available.
      //         if (kalman_initialized) {
      //           u(0,0) = abs((float)pwm_vals[index - 1]) / 175;
      //           // Prediction step only:
      //           Matrix<2,1> mu_p = Ad * mu + Bd * u;
      //           Matrix<2,2> sigma_p = Ad * sigma * (~Ad) + Sigma_u;
      //           mu = mu_p;
      //           sigma = sigma_p;
      //         }
      //     }

      //     // Do not run PID control until the filter has been initialized.
      //     if (!kalman_initialized) {
      //         sensor2.clearInterrupt();
      //         continue; // Skip PID until we have at least one measurement.
      //     }

      //     // Use the estimated position as the current distance.
      //     currentDistance = mu(0,0);
      //     distance = currentDistance;

      //     distances[index] = currentDistance;
      //     time_stamps[index] = now;

      //     ### Perform PID based on distance. ###
      //     index++;
      // }
      // ### Transmit Data ###
      // break;

      case DRIFT:
      {
        distanceSensor1.setIntermeasurementPeriod(20);
        distanceSensor1.setTimingBudgetInMs(20);
        distanceSensor1.startRanging();  //Write configuration bytes to initiate measurement
        Serial.println("Staring Drift!");
        //Build the Array
        int t_0 = millis();
        int distance1;
        float gyrZ;
        float dt_g;
        float dt_0;
        float previous_time_0;
        float setAngle = 0.0;
        float deriv_LPF;
        float alpha = 0.03;
        float p_derivLPF, derivLPF;
        float PID_control_0, end_time, integral_0, endT;
        float target_dist = 304;
        // // This is used to calculate dtin
        // int prev_time = t_0;
        while ((i < ARRAY_LENGTH) && (millis() - t_0 < 10000)) {
          time_data[i] = millis();
          if (distanceSensor1.checkForDataReady()) {
            distance1 = distanceSensor1.getDistance();  //Get the result of the measurement from the sensor
            distanceSensor1.clearInterrupt();
            distance_data1[i] = distance1;
          }
          else {
            distance_data1[i] = distance_data1[i-1];
          }
          error1[i] = distance_data1[i] - setDistance;
          if (myICM.dataReady()) {
            myICM.getAGMT();
            gyrZ = myICM.gyrZ();
            dt_g = ((float) millis() - end_time) / 1000;
            end_time = (float) millis();
            yaw_data[i] = yaw_data[i-1] - gyrZ*dt_g;
          }
          else {
            yaw_data[i] = yaw_data[i-1];
          }
          dt_0 = (millis() - previous_time_0) / 1000;
          error1_0[i] = setAngle - yaw_data[i];
          error_sum = error_sum + (error1_0[i] * dt / 1000);
            if (error_sum > 150) {
              error_sum = 150;
            } else if (error_sum < -150) {
              error_sum = -150;
            }
          i_term = Ki * error_sum;
          deriv_LPF = alpha * gyrZ + (1-alpha)*p_derivLPF;
          p_derivLPF = derivLPF;
          PID_control_0 = Kp*error1_0[i] + Ki*integral_0 - Kd*derivLPF;
          PID_0[i] = PID_control_0;

          if (error1[i] < 0 && setAngle ==0) {
            setAngle = 180;
            endT = millis();
          }
          else if (error1_0[i] >= -3 && error1_0[i] <=3) {
            speedL = baseL;
            speedR = baseR;
            left_motor();
            right_motor();
          }
          else if (error1_0[i] < -3 || error1_0[i] > 3) {
            speedL = baseL - PID_control_0;
            speedR = baseR + PID_control_0;
            left_motor();
            right_motor();
          }

          previous_time_0 = millis();
          i++;
        }
          analogWrite(MotorRightBackward, 0);
          analogWrite(MotorRightForward, 0);
          analogWrite(MotorLeftForward, 0);
          analogWrite(MotorLeftBackward, 0);



          // Start of Control Loop Calculations
        //   current_dist = distance1;
        //   old_error = error1;
        //   dt = millis() - prev_time;
        //   error1 = target_dist - current_dist; //current_dist - target_dist; 
        //   prev_time = millis();
        //   time_data[i] = prev_time;
        //   error_sum = error_sum + (error1 * dt / 1000);
        //   // if (!isnan(error1) && dt > 0) {
        //   //   error_sum += (error1 * dt / 1000.0);
        //   // }
        //   if (error_sum > 150) {
        //     error_sum = 150;
        //   } else if (error_sum < -150) {
        //     error_sum = -150;
        //   }
        //   p_term = Kp * error1;
        //   i_term = Ki * error_sum;
        //   // d_term = 0;
        //   d_term = Kd * (error1 - old_error) / dt;
        //   pwm = p_term + i_term + d_term;

        //   // Control Signal Saturation
        //   if (pwm > maxSpeed) pwm = maxSpeed;
        //   if (pwm < -maxSpeed) pwm = -maxSpeed;

        //   pwm_data[i] = pwm;
        //   if (pwm > 0) {                   //previous code: pwm > 0
        //     analogWrite(MotorRightBackward, 0);          // Kelvin's car: right backward
        //     analogWrite(MotorRightForward, pwm + 5);    // Kelvin's car: right foward
        //     analogWrite(MotorLeftForward, pwm *2.0 + 12);  // Kelvin's car: left forward (PIN3,pwm*2.0 + 12);
        //     analogWrite(MotorLeftBackward, 0);          // Kelvin's car: left backward
        //   } else if (pwm < 0) {            //previous code: pwm < 0
        //     // pwm = abs(pwm);
        //     analogWrite(MotorRightBackward, abs(pwm) + 5);
        //     analogWrite(MotorRightForward, 0);
        //     analogWrite(MotorLeftForward, 0);
        //     analogWrite(MotorLeftBackward, abs(pwm) *2 + 12);  //(PIN2,abs(pwm)*2.0 + 12);
        //   } else {
        //     analogWrite(MotorRightBackward, 0);
        //     analogWrite(MotorRightForward, 0);
        //     analogWrite(MotorLeftForward, 0);
        //     analogWrite(MotorLeftBackward, 0);
        //   }
        // }

        // analogWrite(MotorRightBackward, 0);
        // analogWrite(MotorRightForward, 0);
        // analogWrite(MotorLeftForward, 0);
        // analogWrite(MotorLeftBackward, 0);

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
      case STUNT_GAIN:
      {
        success = robot_cmd.get_next_value(setDistance);  //backward right
        if (!success)
          return;
        success = robot_cmd.get_next_value(baseL);  //forward right
        if (!success)
          return;
        success = robot_cmd.get_next_value(baseR);  //backward left
        if (!success)
          return;
        success = robot_cmd.get_next_value(Kp);  //forward left
        if (!success)
          return;
        success = robot_cmd.get_next_value(Ki);  //forward left
        if (!success)
          return;
        success = robot_cmd.get_next_value(Kd);  //forward left
        if (!success)
          return;

        Serial.print("Stunt gain sent!");
        break;

      }

      case SEND_STUNT_DATA:
      {
        Serial.print("Sending stunt data!");
        for (int j = 0; j< ARRAY_LENGTH; j++) {
          // if(time_data[j] == 0)
          // break;
          tx_estring_value.clear();
          tx_estring_value.append(time_data[j]);
          tx_estring_value.append("|");
          tx_estring_value.append(error1[j]);
          tx_estring_value.append("|");
          tx_estring_value.append(PID_0[j]);
          tx_estring_value.append("|");
          tx_estring_value.append(motorInputR[j]);
          tx_estring_value.append("|");
          tx_estring_value.append(motorInputL[j]);
          tx_estring_value.append("|");
          tx_estring_value.append(distance_data1[j]);
          tx_estring_value.append("|");
          tx_estring_value.append(yaw_data[j]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
        Serial.println("Stunt data arrays sent!");
        break;

      }

      case FLIP:
      {
        // int PIN0_Output, PIN1_Output, PIN2_Output, PIN3_Output;
        // success = robot_cmd.get_next_value(PIN0_Output);  //backward right
        // if (!success)
        //   return;
        // success = robot_cmd.get_next_value(PIN1_Output);  //forward right
        // if (!success)
        //   return;
        // success = robot_cmd.get_next_value(PIN2_Output);  //backward left
        // if (!success)
        //   return;
        // success = robot_cmd.get_next_value(PIN3_Output);  //forward left
        // if (!success)
        //   return;

        // Serial.print("Pins output: ");
        // Serial.print(PIN0_Output);
        // Serial.print(", ");
        // Serial.println(PIN1_Output);
        // Serial.print(", ");
        // Serial.print(PIN2_Output);
        // Serial.print(", ");
        // Serial.println(PIN3_Output);

        //  analogWrite(PIN0, 0);
        //  analogWrite(PIN1, 80);
        //  analogWrite(PIN2, 0);
        //  analogWrite(PIN3, 70);
        //  delay(2000);

          // analogWrite(MotorLeftForward, PIN0_Output);
          // analogWrite(MotorRightForward, PIN2_Output);
          // analogWrite(MotorRightBackward, PIN3_Output);
          // delay(2000);
          // // 1. Go forward for 2 seconds
          
          // // analogWrite(MotorLeftForward, 100);
          // // analogWrite(MotorLeftBackward, 0);
          // // analogWrite(MotorRightForward, 100);
          // // analogWrite(MotorRightBackward, 0);
          // // delay(5000);

          // // 2. Spin 180 degrees (adjust time to match your robot's turning speed)
          // analogWrite(MotorLeftForward, 200);
          // analogWrite(MotorLeftBackward, 0);
          // analogWrite(MotorRightForward, 0);
          // analogWrite(MotorRightBackward, 200);
          // delay(700);  // Approximate 180° t
          // urn

          // // 3. Go forward again (back to start point, facing backward)
          // analogWrite(MotorLeftForward, 150);
          // analogWrite(MotorLeftBackward, 0);
          // analogWrite(MotorRightForward, 150);
          // analogWrite(MotorRightBackward, 0);
          // delay(2000);

          // // 4. Stop
          // analogWrite(MotorLeftForward, 0);
          // analogWrite(MotorLeftBackward, 0);
          // analogWrite(MotorRightForward, 0);
          // analogWrite(MotorRightBackward, 0);

          // distanceSensor1.startRanging();
          // float distance = distanceSensor1.getDistance(); 
          // distanceSensor1.clearInterrupt();
          // distanceSensor1.stopRanging();
          // if (distance > 925.0) {
          //   forward();
          // }
          // else {
          //   counter++;
          //   if (counter==1){
          //   stop();
          //   backward();
          // }
          //   else{
          //     backward();
          // }
          // } 

          
        break;
      }

      case LAB8_STUNT:
      {
          currentMillis = millis();
          previousMillis = millis();

          while (1)
          {
              tx_estring_value.clear();
              int time = millis();

              distanceSensor1.startRanging();
              while (!distanceSensor1.checkForDataReady()) {
                  delay(1);
              }

              int distance1 = distanceSensor1.getDistance(); // Get distance from sensor
              distanceSensor1.clearInterrupt();
              distanceSensor1.stopRanging();

              if (distance1 >= 925)
              {
                  tx_estring_value.append(time);
                  tx_estring_value.append("\r");
                  tx_estring_value.append(distance1);
                  tx_characteristic_string.writeValue(tx_estring_value.c_str());
                  tx_estring_value.clear();

                  // Forward motion
                  analogWrite(MotorLeftForward, 0);
                  analogWrite(MotorLeftBackward, 254);
                  analogWrite(MotorRightForward, 0);
                  analogWrite(MotorRightBackward, 254);
              }
              else
              {
                  // Backward (flip) motion
                  analogWrite(MotorLeftForward, 255);
                  analogWrite(MotorLeftBackward, 0);
                  analogWrite(MotorRightForward, 255);
                  analogWrite(MotorRightBackward, 0);
                  delay(1200);

                  // Stop
                  analogWrite(MotorLeftForward, 255);
                  analogWrite(MotorLeftBackward, 255);
                  analogWrite(MotorRightForward, 255);
                  analogWrite(MotorRightBackward, 255);
                  delay(500);

                  // Quick forward nudge
                  analogWrite(MotorLeftForward, 0);
                  analogWrite(MotorLeftBackward, 200);
                  analogWrite(MotorRightForward, 0);
                  analogWrite(MotorRightBackward, 100);
                  delay(2000);

                  break; // exit loop after flip
              }
          }

          // Final full stop
          analogWrite(MotorLeftForward, 255);
          analogWrite(MotorLeftBackward, 255);
          analogWrite(MotorRightForward, 255);
          analogWrite(MotorRightBackward, 255);
          break;
      }
      
      case LAB8:
      {
        flip();

        }


      default:
        Serial.print("Invalid Command Type: ");
        Serial.println(cmd_type);
        break;
    }
}

void flip() {
  #include <BasicLinearAlgebra.h>
  using namespace BLA;
  int pwm;
  int t;
  pwm = 200;
  int distance2;
  while (distance2 > 1200) {
    if (distanceSensor1.checkForDataReady()) {
      distance2 = distanceSensor1.getDistance();
      distanceSensor1.clearInterrupt();
      distanceSensor1.stopRanging();
      distanceSensor1.startRanging();
      distance_data1[i] = distance2;
      time_data[i] = (float) millis();
      pwm_data[i] = 255;
    }
    driveStraightwithCalib2(1, 255);
    i++;
  }

  driveStraightwithCalib2(-1, 255);
  pwm_data[i] = -255;
  time_data[i] = (float) millis();
  delay(1000);
  i++;

  driveStraightwithCalib2(-1, 100);
  pwm_data[i] = -150;
  time_data[i] = (float) millis();
  delay(2000);

  driveStraightwithCalib2(-1, 0);
  pwm_data[i] = 0;
  time_data[i] = (float) millis();
}

void driveStraightwithCalib2(int direction, int pwm) {
  if (direction ==-1) {
    analogWrite(MotorLeftForward, pwm * 2 + 12 );
    analogWrite(MotorLeftBackward, 0);
    analogWrite(MotorRightForward, pwm+5);
    analogWrite(MotorRightBackward, 0);
  } else {
    analogWrite(MotorLeftForward, 0);
    analogWrite(MotorLeftBackward, pwm * 2 + 12);
    analogWrite(MotorRightForward, 0);
    analogWrite(MotorRightBackward, pwm+5);
  }
  //stop//
  analogWrite(MotorLeftForward,0);
  analogWrite(MotorLeftBackward, 0);
  analogWrite(MotorRightForward, 0);
  analogWrite(MotorRightBackward, 0);
}

// Function to move Right Motor
void right_motor() {
  int lower = 50;

  // Pin 9 = Right, Forward
  if (speedR > 0) {
    if (speedR >= lower && speedR <= 255) {
      analogWrite(9, speedR);
      analogWrite(7, 0);
      motorInputR[i] = speedR;
    }
    else if (speedR >= 0 && speedR < lower) {
      analogWrite(9, lower);
      analogWrite(7, 0);
      motorInputR[i] = lower;
    }
    else if (speedR > 255) {
      analogWrite(9, 255);
      analogWrite(7, 0);
      motorInputR[i] = 255;
    }
  }

  // Pin 7 = Right, Backward
  if (speedR < 0) {
    if (abs(speedR) >= lower && abs(speedR) <= 255) {
      analogWrite(9, 0);
      analogWrite(7, abs(speedR));
      motorInputR[i] = -speedR;
    }
    else if (abs(speedR) > 0 && abs(speedR) < lower) {
      analogWrite(9, 0);
      analogWrite(7, lower);
      motorInputR[i] = -lower;
    }
    else if (abs(speedR) > 255) {
      analogWrite(9, 0);
      analogWrite(7, 255);
      motorInputR[i] = -255;
    }
  }
}

//move left motor
void left_motor() {
  int lower = 50;

  // Pin 9 = Right, Forward
  if (speedL > 0) {
    if (speedL >= lower && speedL <= 255) {
      analogWrite(9, speedL);
      analogWrite(7, 0);
      motorInputL[i] = speedL;
    }
    else if (speedL >= 0 && speedL < lower) {
      analogWrite(9, lower);
      analogWrite(7, 0);
      motorInputL[i] = lower;
    }
    else if (speedL > 255) {
      analogWrite(9, 255);
      analogWrite(7, 0);
      motorInputL[i] = 255;
    }
  }

  // Pin 11 = Left, Backward
  if (speedL < 0) {
    if (abs(speedL) >= lower && abs(speedL) <= 255) {
      analogWrite(9, 0);
      analogWrite(7, abs(speedL));
      motorInputL[i] = -speedL;
    }
    else if (abs(speedL) > 0 && abs(speedL) < lower) {
      analogWrite(9, 0);
      analogWrite(7, lower);
      motorInputL[i] = -lower;
    }
    else if (abs(speedL) > 255) {
      analogWrite(9, 0);
      analogWrite(7, 255);
      motorInputL[i] = -255;
    }
  }
}


void setup() {
  analogWrite(MotorRightBackward,0); //Kelvin's car
  analogWrite(MotorRightForward,0);
  analogWrite(MotorLeftForward,0);
  analogWrite(MotorLeftBackward,0);
  pinMode(PIN0, OUTPUT);
  pinMode(PIN3, OUTPUT);
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
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again..");
      delay(500);
    } else {
      initialized = true;
    }
  }
  bool success = true;  // In order to confirm that the DMP configuration was successful
  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 4) == ICM_20948_Stat_Ok);  // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success) {
    Serial.println("DMP success!");
  } else {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ;  // Do nothing more
  }

  //   }
  pinMode(SHUTDOWN_PIN, OUTPUT);

  // Set the address of TOF1 to 0xF0
  digitalWrite(SHUTDOWN_PIN, LOW);  // Shut down TOF2
  delay(10);
  distanceSensor0.setI2CAddress(0xF0);
  delay(10);
  digitalWrite(SHUTDOWN_PIN, HIGH);  // Restart TOF2
  delay(10);

  if (distanceSensor0.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 0 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");  //uncomment later
  if (distanceSensor1.begin() != 0)  //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  Serial.println(distanceSensor0.getI2CAddress());
  Serial.println(distanceSensor1.getI2CAddress());  //uncomment later
  distanceSensor0.setDistanceModeLong();            //Write configuration bytes to initiate measurement
  distanceSensor1.setDistanceModeLong();            //Write configuration bytes to initiate measurement //uncomment later
  // distanceSensor0.setTimingBudgetInMs(33);
  // distanceSensor1.setTimingBudgetInMs(33);
  // measure periodically. Intermeasurement period must be >/= timing budget.
  distanceSensor0.setIntermeasurementPeriod(40);
  distanceSensor1.setIntermeasurementPeriod(40);  //uncomment later
                                                  //RIGHT MOTOR
                                                  // pinMode(MotorRightForward, OUTPUT);
                                                  // pinMode(MotorRightBackward, OUTPUT);
                                                  // //LEFT MOTOR
                                                  // pinMode(MotorLeftForward, OUTPUT);
                                                  // pinMode(MotorLeftBackward, OUTPUT); //11
}

void write_data() {
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

void read_data() {
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written()) {
    handle_command();
  }
}


void forward(){
  analogWrite(MotorLeftForward, 210);
  analogWrite(MotorLeftBackward, 0);
  analogWrite(MotorRightForward, 200);
  analogWrite(MotorRightBackward, 0);
}
void backward(){
  analogWrite(MotorLeftForward, 0);
  analogWrite(MotorLeftBackward, 255);
  analogWrite(MotorRightForward, 0);
  analogWrite(MotorRightBackward, 250);
}
void stop(){
  analogWrite(MotorLeftForward, 255);
  analogWrite(MotorLeftBackward, 255);
  analogWrite(MotorRightForward, 255);
  analogWrite(MotorRightBackward, 255);
}


void loop() {
  // Listen for connections
  // stunt_func();
  // delay(100);
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
