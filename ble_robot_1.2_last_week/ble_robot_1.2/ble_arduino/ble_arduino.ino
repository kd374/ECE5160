

#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <ICM_20948.h>
#include <Wire.h>
#define AD0_VAL   1
#define XSHUT 8
#define ADDRESS 0x30
#include "SparkFun_VL53L1X.h"

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "03cee772-2e79-410b-8846-ce32c73d3bfd"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

//////////// BLE UUIDs ////////////

/////////// Lab 2 ////////////
#define SERIAL_PORT Serial
#define ARRAY_LENGTH 1000
#define ARRAY_LENGTH2 1000
ICM_20948_I2C myICM;



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

unsigned long start_time = millis();
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
const int data_array = 128; 
int time_data[data_array];
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
int last_time = 0;
// float roll_g[ARRAY_LENGTH2] = {0.0};
float pitch_g = 0;
// float yaw_g[ARRAY_LENGTH2] = {0.0};
float roll_g = 0;
float yaw_g = 0;
float roll_g_array[ARRAY_LENGTH2];
float pitch_g_array[ARRAY_LENGTH2];
float yaw_g_array[ARRAY_LENGTH2];
float distance[ARRAY_LENGTH];
float dt = 0;
bool send_IMU_data = false;
bool send_IMU_data2 = false;
int IMU_entries_gathered_acc = 0;
int IMU_entries_gathered_gyro = 0;
bool RECORD_IMU = false;

/////////////LAB 3//////////////
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, XSHUT);
unsigned long starting_time;
unsigned long e_time;
float time_data_2tof[ARRAY_LENGTH];
float distance_data1[ARRAY_LENGTH];
float distance_data2[ARRAY_LENGTH];
void collectIMUData_ACC();
void collectIMUData_GYRO();
void collect2TOFData();


//////////////LAB 4//////////////////
#define PIN0 0 //A1 IN / B1 IN
#define PIN1 1 //A2 IN / B2 IN
#define PIN2 2 //A1 IN / B1 IN
#define PIN3 3 //A2 IN / B2 IN

//////////////LAB 5///////////////
//PID Control
// int start_pid; //do we start the cmd for pid ctrl?
// //gain vals for PID controller
// float Kp_val;
// float Kd_val;
// float Ki_val;
// //target distance
// float target_dist;
// //collect PID data
// float kp_pwm[500];
// float ki_pwm[500];
// float error_val[500];
// //PWM Signal
// float pwm;
// const int n =1;
// int m = 0;

// float tof_1[1700];
// float tof_2[1700];
// unsigned int time_imu_data[1700];

//////////////LAB 5 UPDATED//////////////////
float current_pos;
float target_pos;
float Kd, Kp, Ki;
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



enum CommandTypes
{
    PING,
    SEND_TWO_INTS,
    SEND_THREE_FLOATS,
    ECHO,
    DANCE,
    SET_VEL,
    GET_TIME_MILLIS,
    TIME_LOOP,
    SEND_TIME_DATA, 
    GET_TEMP_READINGS,
    CALCULATE_DATA_RATE,
    COLLECT_IMU_DATA_ACC,
    SEND_IMU_DATA_ACC,
    SEND_IMU_DATA_GYRO, 
    START_IMU_DATA,
    STOP_IMU_DATA,
    TIME_OF_FLIGHT,
    GATHER_2TOF_AND_IMU_DATA, 
    CALIBRATION, 
    // START_PID,
    // SEND_PID_CONTROL_DATA 
    PID_POSITION_CONTROL
};

// Lab 2 - Accelerometer
//Convert to pitch data (pitch = theta = y rotation)
//Return a float in degrees

void setup() {
    //digitalWrite(XSHUT, LOW);
    pinMode(PIN0, OUTPUT);
    pinMode(PIN1, OUTPUT);
    pinMode(PIN2, OUTPUT);
    pinMode(PIN3, OUTPUT);
    BLE.begin();
    Wire.begin();
    Serial.begin(115200);
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

    Wire.setClock(400000);
    bool initialized = false;
    while( !initialized ) {
      myICM.begin( Wire, AD0_VAL );
      Serial.print( F("Initialization of the sensor returned: ") );
      Serial.println( myICM.statusString() );
      if( myICM.status != ICM_20948_Stat_Ok ){
        Serial.print("My ICM status:");
        Serial.print(myICM.status);
        Serial.println( "Trying again.." );
        delay(500);
      } else{
        initialized = true;
      }
    }
    Serial.println("VL53L1X Qwiic Test");
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);
    distanceSensor1.setI2CAddress(ADDRESS);
    Serial.print("The new distance for sensor 1 address: 0x");
    Serial.println(distanceSensor1.getI2CAddress(), HEX);
    if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
      while (1);
    }
    digitalWrite(XSHUT, HIGH);
    Serial.print("The distance sensor 2 address: 0x");
    Serial.println(distanceSensor2.getI2CAddress(), HEX);
     if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
      while (1);
    }
    Serial.println("Sensors 1 and 2 are online!");
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void handle_command()
{   
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;
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
            float float_a, float_b, float_c;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;

            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            Serial.print("Three floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.println(float_c);
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
             tx_estring_value.clear();
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
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
        
        case GET_TIME_MILLIS: 
        
            int new_millis;
            new_millis = (int) millis();
            tx_estring_value.clear();
            tx_estring_value.append("T: ");
            tx_estring_value.append(new_millis);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Sent back: ");
            Serial.println(new_millis);
  
            break;

        case TIME_LOOP: {
            unsigned long start_time = millis(); //unsigned long start_time = millis();
            int counter = 0;
            while ((millis()-start_time) < 7000) { //collecting for 7 seconds
              tx_estring_value.clear();
              tx_estring_value.append("T: ");
              tx_estring_value.append((int) millis());
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
              counter ++;
            }

            delay(2000);
            tx_estring_value.clear();
            tx_estring_value.append(counter);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Current time sent back: ");
            Serial.println(millis());
            Serial.print("Number of timestamps: ");
            Serial.println(counter);
        }
    
            break;

        case SEND_TIME_DATA:
        {
            int n = 0; 
            int start_time = millis();
            while (n < data_array) {
              time_data[n] = (int) millis();
              n++;
              if (n == (data_array-1))
                Serial.println("Not enough memory.");
            }
            for (int i = 0; i < data_array; i++) {
              if (time_data[i] == 0)
                break;
            
              tx_estring_value.clear();
              tx_estring_value.append("For sample ");
              tx_estring_value.append(i);
              tx_estring_value.append(": ");
              tx_estring_value.append(time_data[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
              Serial.println("The data is sent");
              break;
        }

        case GET_TEMP_READINGS:
        {
            int n = 0; 
            int start_time = millis();
            while (n < data_array) {
              time_data[n] = (int) millis();
              temperature_data[n] = getTempDegF();
              n++;
              if (n == (data_array-1))
                Serial.println("Not enough memory.");
            }
            for (int i = 0; i < data_array; i++) {
              if (time_data[i] == 0)
                break;
            
              tx_estring_value.clear();
              tx_estring_value.append("For sample ");
              tx_estring_value.append(i);
              tx_estring_value.append(": The temperature is: ");
              tx_estring_value.append(temperature_data[i]);
              tx_estring_value.append(" °F and is during time ");
              tx_estring_value.append(time_data[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
              Serial.print("Data sent back:");
              Serial.println(tx_estring_value.c_str());
            }
              // Serial.print("Data sent back:");
              // Serial.println(tx_estring_value.c_str());
              break;
        }
        case CALCULATE_DATA_RATE: 
        {
            int sent_bytes[] = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120};
            String receivedData = (char*)rx_characteristic_string.value();  // Extract received data

            int data_size = receivedData.toInt();  // Convert received string to integer

            tx_estring_value.clear();
            tx_estring_value.append("Data Size: ");
            tx_estring_value.append(data_size);
    
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());

            break;
        }
        // case COLLECT_IMU_DATA:
        //     if (IMU_entries_gathered_acc >= ARRAY_LENGTH) {
        //       send_IMU_data = true;
        //       return;
        //     }
        //     if (myICM.dataReady()) {
        //       myICM.getAGMT();

        //       time_array_acc[IMU_entries_gathered_acc] = millis();
        //       pitch_a[IMU_entries_gathered_acc] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
        //       roll_a[IMU_entries_gathered_acc] = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
        //       IMU_entries_gathered_acc++;
        //     }
        //     break;
        case SEND_IMU_DATA_ACC:
            Serial.println("Collecting IMU Accelerator data... ");
            while (IMU_entries_gathered_acc < ARRAY_LENGTH) {
              collectIMUData_ACC();
              //filterIMUData_ACC();
            }
            Serial.println("Colleting IMU Accelerator data complete! Now sending...");
            if (IMU_entries_gathered_acc == 0) {
              Serial.println("No IMU data to send.");
              break;
            }
            for (int i = 0; i < ARRAY_LENGTH; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("Time: ");
              tx_estring_value.append(time_array_acc[i]);
              tx_estring_value.append("Pitch: ");
              tx_estring_value.append(pitch_a[i]);
              tx_estring_value.append("Roll: ");
              tx_estring_value.append(roll_a[i]);
              tx_estring_value.append("Pitch filtered: ");
              tx_estring_value.append(pitch_a_acc_lpf[i]);
              tx_estring_value.append("Roll filtered: ");
              tx_estring_value.append(roll_a_acc_lpf[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            } 
            IMU_entries_gathered_acc = 0;
            send_IMU_data = false;

            break;
        case SEND_IMU_DATA_GYRO:
            Serial.println("Collecting IMU Gyroscope data... ");
            while (IMU_entries_gathered_gyro < ARRAY_LENGTH2) {
              collectIMUData_GYRO();
              //filterIMUData_ACC();
            }
            Serial.println("Colleting IMU Gyroscope data complete! Now sending...");
            if (IMU_entries_gathered_gyro == 0) {
              Serial.println("No IMU data to send.");
              break;
            }
            for (int i = 0; i < ARRAY_LENGTH2; i++) {
              tx_estring_value.clear();
              tx_estring_value.append("Time: ");
              tx_estring_value.append((float) time_array_gyro[i]);
              tx_estring_value.append("Pitch: ");
              tx_estring_value.append(pitch_g_array[i]);
              tx_estring_value.append("Roll: ");
              tx_estring_value.append(roll_g_array[i]);
              tx_estring_value.append("Yaw: ");
              tx_estring_value.append(yaw_g_array[i]);
              tx_estring_value.append("Pitch filtered: ");
              tx_estring_value.append(complementary_pitch[i]);
              tx_estring_value.append("Roll filtered: ");
              tx_estring_value.append(complementary_roll[i]);
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }
            IMU_entries_gathered_gyro = 0;
            send_IMU_data2 = false;
            break; 

        case START_IMU_DATA:
            Serial.println("Recording IMU data...");
            digitalWrite(LED_BUILTIN, HIGH);
            RECORD_IMU = true;
            break;

        case STOP_IMU_DATA:
            RECORD_IMU = false;
            digitalWrite(LED_BUILTIN, LOW);
            Serial.println("Stopped recording IMU data");
            break;

        case TIME_OF_FLIGHT:
            Serial.println("VL53L1X Qwiic Test");
            distanceSensor1.setDistanceModeShort();
            Serial.println("Sensor is online!");

            float distance[10], dt[10];

              for (int i = 0; i < 10; i++) {
                distanceSensor1.startRanging();
                starting_time = millis();
                unsigned long timeout = millis();
                while (!distanceSensor1.checkForDataReady()) {
                  if (millis() - timeout > 500) {
                    Serial.println("Sensor timeout!");
                    break;
                  }
                }
                distance[i] = (distanceSensor1.getDistance())/10.0;

                distanceSensor1.clearInterrupt();
                distanceSensor1.stopRanging();
                dt[i] = millis() - starting_time;
              }
              Serial.println("Sending data...");
              for (int i = 0; i<10; i++) {
                tx_estring_value.clear();
                // tx_estring_value.append("Time: ");
                tx_estring_value.append(dt[i]);
                tx_estring_value.append("|");
                tx_estring_value.append(distance[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                Serial.print("Time: ");
                Serial.print(dt[i]);
                Serial.print("Distance: ");
                Serial.println(distance[i]);
              }
              Serial.print("Sensor done!");
              break;

        // case 2_TOF_AND_IMU_DATA_SEND:
        //      //in here, collect data
        //     start_time = millis();
        //     // if(myICM.dataReady()){
        //     i = 0;
        //     Serial.print("Start execution");
        //     while((i < ARRAY_LENGTH) and ((millis()-start_time) < 90000 )){
           

        //     // collect CF pitch and roll
        //       myICM.getAGMT();
        //       dt = (micros() - last_time) / 1000000.0; // Time difference in seconds
        //       last_time = micros();

        //     ////////CF Pitch//////////
        //       dt = (micros() - last_time) / 1000000.0; // Time difference in seconds
        //       last_time = micros();
              
        //     // Calculate the gyroscope-based angle (integrated from angular velocity)
        //       pitch_g = myICM.gyrX();
        //     // Calculate the accelerometer-based pitch angle
        //       pitch_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
        //       if (x == 0) {
        //         cf_pitch[i] = pitch_g * (1 - alpha_cf_pitch) + pitch_a * alpha_cf_pitch;
        //       } else {
        //         cf_pitch[i] = cf_pitch[i-1] * (1 - alpha_cf_pitch) + pitch_a * alpha_cf_pitch;
        //       }

        //       roll_g = myICM.gyrY();
        //     // Calculate the accelerometer-based roll angle
        //       roll_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
        //     // Apply the complementary filter
        //       if (x == 0) {
        //         cf_roll[i] = roll_g * (1 - alpha_cf_roll) + roll_a * alpha_cf_roll;
        //       } else {
        //         cf_roll[i] = cf_roll[i-1] * (1 - alpha_cf_roll) + roll_a * alpha_cf_roll;
        //       }

        //     //ToF1 and ToF2 data 
        //       distanceSensor1.startRanging(); // Write configuration bytes to initiate measurement
        //       tof1[i] = distanceSensor1.getDistance();
        //       distanceSensor1.clearInterrupt();
        //       distanceSensor1.stopRanging();
        //       distanceSensor2.startRanging(); // Write configuration bytes to initiate measurement
        //       tof2[i] = distanceSensor2.getDistance();
        //       distanceSensor2.clearInterrupt();
        //       distanceSensor2.stopRanging();

        //     //record the time
        //       time_imu_data[i] = (unsigned int) millis();
        //       i++;

        //     }

        //     Serial.print("Finished collecting data");

        //     break;

        case GATHER_2TOF_AND_IMU_DATA:
            //IMU data from Lab 2
            Serial.println("Collecting IMU Accelerator data... ");
            while (IMU_entries_gathered_acc < ARRAY_LENGTH) {
              collectIMUData_ACC();
              collect2TOFData();
            }
            Serial.println("Colleting IMU Accelerator data complete! Now sending...");
            if (IMU_entries_gathered_acc == 0) {
              Serial.println("No IMU data to send.");
              break;
            }
            for (int i = 0; i < ARRAY_LENGTH; i++) {
              tx_estring_value.clear();
              tx_estring_value.append(time_array_acc[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(pitch_a[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(roll_a[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(time_data_2tof[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(distance_data1[i]);
              tx_estring_value.append("|");
              tx_estring_value.append(distance_data2[i]);
              Serial.println(tx_estring_value.c_str());
              tx_characteristic_string.writeValue(tx_estring_value.c_str());
            } 
            IMU_entries_gathered_acc = 0;
            send_IMU_data = false;
            break;

        case CALIBRATION:
            int PIN0_Output, PIN1_Output, PIN2_Output, PIN3_Output;
            success = robot_cmd.get_next_value(PIN0_Output);
            if (!success)
                return;
            success = robot_cmd.get_next_value(PIN1_Output);
            if (!success)
                return;
            success = robot_cmd.get_next_value(PIN2_Output);
            if (!success)
                return;
            success = robot_cmd.get_next_value(PIN3_Output);
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

            analogWrite(PIN0, 0);
            analogWrite(PIN1, 78);
            analogWrite(PIN2, 0);
            analogWrite(PIN3, 70);
            delay(2000);
            analogWrite(PIN0, PIN0_Output);
            analogWrite(PIN1, PIN1_Output);
            analogWrite(PIN2, PIN2_Output);
            analogWrite(PIN3, PIN3_Output);
            delay(1500);
            analogWrite(PIN0, 0);
            analogWrite(PIN1, 0);
            analogWrite(PIN2, 0);
            analogWrite(PIN3, 0);
            break;

        // case START_PID:
        //   Serial.println("Got into start_pid case");
        //   // success = robot_cmd.get_next_value(start_pid);
        //   // if(!success) return;

        //   //Kp value
        //   success = robot_cmd.get_next_value(Kp_val);
        //   if (!success) return;
          
        //   // // //Kd value
        //   // // success = robot_cmd.get_next_value(Kd_val);
        //   // // if (!success) return;

        //   // // //Ki value
        //   // // success = robot_cmd.get_next_value(Ki_val);
        //   // // if (!success) return;

        //   //Target Distance
        //   success = robot_cmd.get_next_value(target_dist);
        //   if (!success) return;

        //   // Serial.print("Got Kp_val: ");
        //   // Serial.println(Kp_val, 5);//second argument tells how decimal places to print out

        //   // if(Kp_val != 0){
        //   //   Serial.println("Started PID control");
        //   //   Serial.print("Got Kp_val: ");
        //   //   Serial.println(Kp_val, 5);//second argument tells how decimal places to print out
        //   // }

        //   // Serial.print("start_pid: ");
        //   // Serial.println(start_pid);
        //   Serial.print("Kp: ");
        //   Serial.println(Kp_val);
        //   // // Serial.print("Kd: ");
        //   // // Serial.println(Kd_val);
        //   // // Serial.print("Ki: ");
        //   // // Serial.println(Ki_val);
        //   Serial.print("Target Distance: ");
        //   Serial.println(target_dist);

        //   m = 0;
        //   // Serial.print("Start execution");
        //   // // for proportional control

        //   //reset start_time each time you run the cmd
        //   start_time = millis();

        //   //reset the arrays so it will be set to 0 at start
        //   memset(tof_1, 0, sizeof(tof_1));
        //   memset(error_val, 0, sizeof(error_val));
        //   memset(kp_pwm, 0, sizeof(kp_pwm));
        //   memset(time_imu_data, 0, sizeof(time_imu_data));

        //   while((m < 500) and ((millis()-start_time) < 30000 )){

        //     distanceSensor1.startRanging();
        //     tof_1[m] = distanceSensor1.getDistance();
        //     // Serial.print("Distance: ");
        //     // Serial.println(tof_1[m]);


        //     // //look at difference btwn target distance and measured dist
        //     error_val[m] = tof_1[m] - target_dist ;
        //     // Serial.print("Error: ");
        //     // // Serial.println(error_val[m]);

        //     // //duty cycle to give to the motors
        //     kp_pwm[m] = Kp_val * error_val[m]; //make a factor so get value between 0 and 255
            
        //     if(m == 250){
        //       Serial.print("PWM: ");
        //       Serial.println(kp_pwm[m]);

        //     }

        //     if(error_val[m]>= 0){
        //       forward_motion(kp_pwm[m]);
        //     }
        //     else{
        //       backwards_motion(kp_pwm[m]);
        //     }
           
        //     // // //   // dt = millis() - start_time;
        //     // // //   // // for integral control
        //     // // //   // ki_pwm[m] = Ki_val * (error_val[m] * dt);

        //     //   //////Forward////
        //     //   //Motor A
        //     //   //Right Wheels
        //     //   analogWrite(0, kp_pwm[m]);
        //     //   analogWrite(3, 0);
        //     //   //Motor B
        //     //   //Left Wheels
        //     //   analogWrite(1, kp_pwm[m] * 1.45);
        //     //   analogWrite(5, 0);


        //     //record the time
        //     time_imu_data[m] = (unsigned int) millis();
        //     m++;
        //     start_time = millis();

            
        //   }

          //once PID controller is done, stop the car

          //Right Wheels
          // analogWrite(0, 0);
          // analogWrite(3, 0);
          // // Left Wheels
          // analogWrite(1, 0);
          // analogWrite(5, 0);
          // Serial.println("Done");
          // break;

        case PID_POSITION_CONTROL:
          start_time = millis();
          // initialization for Kp, Ki, Kd, Target distance
          success = robot_cmd.get_next_value(Kp);
          if (!success) return;
          success = robot_cmd.get_next_value(Ki);
          if (!success) return;
          success = robot_cmd.get_next_value(Kd);
          if (!success) return;
          success = robot_cmd.get_next_value(target_tof);
          if (!success) return;
          tx_estring_value.clear();
          // Recording the data
          for (int i = 0; i < pid_data_num; i++) {
            distanceSensor2.startRanging();
            pid_time[i] = millis() - start_time;
            pid_tof[i] = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            distanceSensor2.stopRanging();
            pid_speed[i] = pid_position_tof(Kp, Ki, Kd, pid_tof[i], target_tof);
            pid_p[i] = Kp * err;
            pid_i[i] = Ki * integral_err;
            pid_d[i] = Kd * err_d;
          }
          stop(); // Stop the motor and begin to send data
          // Data for time
          tx_estring_value.clear();
          tx_estring_value.append("T");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          for (int i = 0; i < pid_data_num; i ++) {
            tx_estring_value.clear();
            tx_estring_value.append(pid_time[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          // Data for distance
          tx_estring_value.clear();
          tx_estring_value.append("S");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          for (int i = 0; i < pid_data_num; i ++) {
            tx_estring_value.clear();
            tx_estring_value.append(pid_tof[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          // Data for pid signal
          tx_estring_value.clear();
          tx_estring_value.append("C");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          for (int i = 0; i < pid_data_num; i ++) {
            tx_estring_value.clear();
            tx_estring_value.append(pid_speed[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          // Data for pid p propotion
          tx_estring_value.clear();
          tx_estring_value.append("P");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          for (int i = 0; i < pid_data_num; i ++) {
            tx_estring_value.clear();
            tx_estring_value.append(pid_p[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          // Data for pid i propotion
          tx_estring_value.clear();
          tx_estring_value.append("I");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          for (int i = 0; i < pid_data_num; i ++) {
            tx_estring_value.clear();
            tx_estring_value.append(pid_i[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          // Data for pid d propotion
          tx_estring_value.clear();
          tx_estring_value.append("D");
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
          for (int i = 0; i < pid_data_num; i ++) {
            tx_estring_value.clear();
            tx_estring_value.append(pid_d[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }
          break;    
    }
}


int pid_position_tof (float Kp, float Ki, float Kd, float current_pos, float target_pos) {
  current_time = millis();
  dt_time = current_time - prev_time; // in ms
  prev_time = current_time;
  err = current_pos - target_pos;
  err_d = (err - prev_err) / dt_time;
  
  // Wind-up protection
  if (abs(err)<0.01) {
    integral_err = 0;
  }
  else {
    integral_err = integral_err + (err * dt_time);
  }
  if (integral_err > 200) {
    integral_err = 200;
  }
  else if (integral_err < -200) {
    integral_err = -200;
  }
  
  // Calculate speed control signal
  speed_control = (int)(Kp * err + Ki * integral_err + Kd * err_d);
  if (speed_control > max_speed) {
    speed_control = max_speed;
  }
  if (speed_control < (-1 * max_speed)) {
    speed_control = -1 * max_speed;
  }
  if (speed_control > 0) {
    forward(speed_control);
  }
  else if (speed_control < 0) {
    backward(-1 * speed_control);
  }
  prev_err = err;
  return speed_control;
}
void forward(int speed) {
  analogWrite(PIN2,0);  ///AB1IN_LEFT
  analogWrite(PIN3,speed*1.8);  ///AB2IN_LEFT
  analogWrite(PIN0,0);  ///AB1IN_RIGHT
  analogWrite(PIN1,speed);  ///AB2IN_RIGHT
}
void backward(int speed) {
  analogWrite(PIN2,speed*1.8); ///AB1IN_LEFT
  analogWrite(PIN3,0);  ///AB2IN_LEFT
  analogWrite(PIN0,speed); ///AB1IN_RIGHT
  analogWrite(PIN1,0);  ///AB2IN_RIGHT
}
void stop() {
  analogWrite(PIN2,0); 
  analogWrite(PIN3,0);
  analogWrite(PIN0,0); 
  analogWrite(PIN1,0);
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

void loop(void) {
    // Listen for connections
    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());
        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // collectIMUData_GYRO();


            // Read data
            read_data();
        }
        Serial.println("Disconnected");
    }
    /* Computation variables */
    float pitch_a = 0, roll_a = 0, pitch_g = 0.0, roll_g = 0.0, yaw_g = 0.0, dt =0, pitch = 0, roll = 0, yaw = 0;
    float Xm = 0, Ym =0, Zm = 0, x = 0, y = 0;
    unsigned long last_time = millis(); // unsigned long last_time = millis();
    double pitch_a_LPF[] = {0, 0};
    const int n =1;

      // if(myICM.dataReady())
      // {
      //   myICM.getAGMT();                // The values are only updated when you call 'getAGMT'

      // //Slide 20 Accelerometer introduction
      //   Serial.print(", acc_x:"); 
      //   Serial.print( myICM.accX() );
      //   delay(50);
      //   Serial.print(", acc_y:");
      //   Serial.print( myICM.accY() );
      //   delay(50); 
      //   Serial.print(", acc_z:");
      //   Serial.println( myICM.accZ() );
      //   delay(50); 
      

      // //Slide 24, accelerometer
      //     pitch_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI; 
      //     roll_a  = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
      //     Serial.print(", pitch_a:");
      //     Serial.print(pitch_a);
      //     Serial.print(", roll_a:");
      //     Serial.print(roll_a); //FOR THE THIRD DEMO, COMMENT OUT LINE FOR SECOND DEMO
      // }
    // currentMillis = millis();
    // if (currentMillis - previousMillis > interval) {
    //     tx_float_value = tx_float_value + 0.5;
    //     tx_characteristic_float.writeValue(tx_float_value);
    //     if (tx_float_value > 10000) {
    //         tx_float_value = 0;
           
    //     }
    //     previousMillis = currentMillis;
    // }
    // distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
    // while (!distanceSensor1.checkForDataReady())
    // {
    //   delay(0);
    // }
    // int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from the sensor
    // distanceSensor1.clearInterrupt();
    // distanceSensor1.stopRanging();
    // distanceSensor2.startRanging(); //Write configuration bytes to initiate measurement
    // while (!distanceSensor2.checkForDataReady())
    // {
    //   delay(0);
    // }
    // int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from the sensor
    // distanceSensor2.clearInterrupt();
    // distanceSensor2.stopRanging();
    // Serial.print("Current time (ms): ");
    // Serial.print(currentMillis);
    // Serial.print(";");
    // Serial.print(" Distance of sensor 1 (mm): ");
    // Serial.print(distance1);
    // Serial.print(";");
    // Serial.print(" Distance of sensor 2 (mm): ");
    // Serial.print(distance2);
    // Serial.println();
  //   distanceSensor1.startRanging();
  //   distanceSensor2.startRanging();
  
  //  // Get current time in milliseconds
  //   int start_time = millis();

  // // Check the first distance sensor
  // if (distanceSensor1.checkForDataReady()) {
  //   int distance_1 = distanceSensor1.getDistance();
  //   distanceSensor1.clearInterrupt();
  //   Serial.print("Distance of Sensor 1 (mm): ");
  //   Serial.println(distance_1);
  // }

  // // Check the second distance sensor
  // if (distanceSensor2.checkForDataReady()) {
  //   int distance_2 = distanceSensor2.getDistance();
  //   distanceSensor2.clearInterrupt();
  //   Serial.print("Distance of Sensor 2 (mm): ");
  //   Serial.println(distance_2);
  // }

  // // Print the time
  // int end_time = millis();
  // Serial.print("The loop time (ms): ");
  // Serial.println(end_time - start_time);
    
}

 void collectIMUData_ACC() {
    if (IMU_entries_gathered_acc >= ARRAY_LENGTH) {
      send_IMU_data = true;
      return;
    }
    if (myICM.dataReady()) {
      myICM.getAGMT();
      Serial.print("Time: ");
      time_array_acc[IMU_entries_gathered_acc] = millis();
      pitch_a[IMU_entries_gathered_acc] = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
      roll_a[IMU_entries_gathered_acc] = atan2(myICM.accY(),myICM.accZ())*180/M_PI; 
      Serial.print("Pitch: ");
      Serial.print(pitch_a[IMU_entries_gathered_acc]);
      Serial.print("Roll: ");
      Serial.println(roll_a[IMU_entries_gathered_acc]);
      IMU_entries_gathered_acc++;
    }
}

void collect2TOFData() {
    // 2 TOF sensors data
    for (int i = 0; i < ARRAY_LENGTH; i++) {
      time_data_2tof[i] = (int)millis();
      // Distance Sensor 1
      distanceSensor1.startRanging();
      while (!distanceSensor1.checkForDataReady()) {
        delay(0);
      }
      distance_data1[i] = distanceSensor1.getDistance(); 
      distanceSensor1.clearInterrupt();
      distanceSensor1.stopRanging();
      // Distance Sensor 2
      distanceSensor2.startRanging();
      while (!distanceSensor2.checkForDataReady()) {
        delay(0);
      }
      distance_data2[i] = distanceSensor2.getDistance(); 
      distanceSensor2.clearInterrupt();
      distanceSensor2.stopRanging();
      Serial.print("TOF Data - Time: ");
      Serial.print(time_data_2tof[i]);
      Serial.print(" TOF1: ");
      Serial.print(distance_data1[i]);
      Serial.print(" TOF2: ");
      Serial.println(distance_data2[i]);
    }
}

void collectIMUData_GYRO() {
    if (IMU_entries_gathered_gyro >= ARRAY_LENGTH2) {
      send_IMU_data2 = true;
      return;
    }
    while (IMU_entries_gathered_gyro < ARRAY_LENGTH2) {
      if (myICM.dataReady()) {
        myICM.getAGMT();
        // dt = (micros()-last_time)/1000000.0;
        last_time = micros();
        time_array_gyro[IMU_entries_gathered_gyro] = last_time;
        Serial.print("IMU_entries_gathered_gyro: ");
        Serial.print(IMU_entries_gathered_gyro);
        // pitch_g = pitch_g + myICM.gyrX()*dt;
        pitch_g_array[IMU_entries_gathered_gyro] = pitch_g;
        // roll_g = roll_g + myICM.gyrY()*dt;
        roll_g_array[IMU_entries_gathered_gyro] = roll_g;
        // yaw_g = yaw_g + myICM.gyrZ()*dt;
        yaw_g_array[IMU_entries_gathered_gyro] = yaw_g;
        Serial.print(", pitch_g:");
        Serial.print(pitch_g_array[IMU_entries_gathered_gyro]);
        Serial.print(", roll_g:");
        Serial.print(roll_g_array[IMU_entries_gathered_gyro]);
        Serial.print(", yaw_g:");
        Serial.println(yaw_g_array[IMU_entries_gathered_gyro]);
        IMU_entries_gathered_gyro++;
        delay(20);
      }
    }
}

// void filterIMUData_ACC() {
//   float alpha = 0.95;
//   pitch_a_acc_lpf[0] = pitch_a[0];
//   roll_a_acc_lpf[0] = roll_a[0];
//   for (int n = 1; n < ARRAY_LENGTH; n++) {
//     const float acc_alpha = 0.2;
//     pitch_a_acc_lpf[n] = acc_alpha*pitch_a[n] + (1-acc_alpha)*pitch_a_acc_lpf[n-1];
//     pitch_a_acc_lpf[n-1] = pitch_a_acc_lpf[n];
//     roll_a_acc_lpf[n] = acc_alpha*roll_a[n] + (1-acc_alpha)*roll_a_acc_lpf[n-1];
//     roll_a_acc_lpf[n-1] = roll_a_acc_lpf[n];

//     complementary_pitch[n] = 0.2 * pitch_g_array[n] + 0.8* pitch_a_acc_lpf[n];
//     complementary_roll[n] = 0.2 * roll_g_array[n] + 0.8 * roll_a_acc_lpf[n];
//     complementary_pitch[n] = 0.18*pitch_a[n] + (1-0.18)*pitch_a_acc_lpf[n-1];
//     complementary_pitch[n-1] = complementary_pitch[n];
//     complementary_roll[n] = 0.18*roll_a[n] + (1-0.18)*roll_a_acc_lpf[n-1];
//     complementary_roll[n-1] = complementary_roll[n];
//   }
// }

// void filterIMUData_GYRO() {
//   pitch_a_gyro_lpf[0] = pitch_a[0];
//   roll_a_gyro_lpf[0] = roll_a[0];

//   for (int n = 1; n < ARRAY_LENGTH2; n++) {
//     const float alpha = 0.2;
//     pitch_a_gyro_lpf[n] = alpha*pitch_a[n] + (1-alpha)*pitch_a_gyro_lpf[n-1];
//     pitch_a_gyro_lpf[n-1] = pitch_a_gyro_lpf[n];
//     roll_a_gyro_lpf[n] = alpha*roll_a[n] + (1-alpha)*roll_a_gyro_lpf[n-1];
//     roll_a_gyro_lpf[n-1] = roll_a_gyro_lpf[n];
//   }
// }

//////////////////PREVIOUS CODE////////////////////

// void forward_motion(float PWM){
//     //Right Wheels
//     analogWrite(0, PWM);
//     analogWrite(1, 0);
//     // //Motor B
//     // //Left Wheels
//     analogWrite(2, PWM);
//     analogWrite(3, 0);
// }

// //backwards motion
// void backwards_motion(float PWM){
//     //Right Wheels
//     analogWrite(0, abs(PWM));  ///right
//     analogWrite(1, 0);  ///right 
//     // //Motor B
//     // //Left Wheels
//     analogWrite(2, abs(PWM));  ///left backwards
//     analogWrite(3, 0);  ///left forwards 
// }
