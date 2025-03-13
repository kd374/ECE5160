
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
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);

//////////// BLE UUIDs ////////////
// #define BLE_UUID_TEST_SERVICE "fc457481-eb77-45bf-8ae5-ebad49aa0dce"
#define BLE_UUID_TEST_SERVICE "03cee772-2e79-410b-8846-ce32c73d3bfd"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define MotorLeftForward 9
#define MotorLeftBackward 11
#define MotorRightForward 13
#define MotorRightBackward 12

//////////// BLE UUIDs ////////////

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

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long startTime = 0;

float Kp = 0;
float Ki = 0;
float Kd = 0;

float maxSpeed = 255;


const int ARRAY_LENGTH = 200;
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

// float pitch_comp_data[ARRAY_LENGTH];
// float roll_comp_data[ARRAY_LENGTH];

int distance_data0[ARRAY_LENGTH];
int distance_data1[ARRAY_LENGTH];

float pwm_data[ARRAY_LENGTH];
int fast_time[ARRAY_LENGTH];

int i = 0;
unsigned long last_time = millis();
double targetYaw = 0;
double currYaw;
float dt = 0;
float error_pid;
float pwm;
float sumError;
float Pterm = 0.001;
float Iterm = 0.0;
float Dterm = 0.0;
float old_error;

  //Build the Array
unsigned long start_time = millis();

bool PID_STRT;
//////////// Global Variables ////////////

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
    START_PID,
    CHANGE_GAIN,
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

          case PID_START:
          {
            int i = 0;
            int k = 0;
            float pitch_g = 0, roll_g = 0, yaw_g = 0, dt =0, pitch_g_accum = 0, roll_g_accum = 0, yaw_g_accum = 0;
            unsigned long last_time = millis();
            unsigned long last_time1 = millis();
            const float alpha = 0.2;
            int distance1 = 0;
            float current_distance;
            float error1;
            float target_distance = 304;
            float pwm;
            float error_sum;
            float p_term;
            float i_term;
            float d_term;
            float old_error;

            distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement

            Serial.println("STARTING PID");
              //Build the Array

              int t_0 = millis();
              // This is used to calculate dtin
              int prev_time = t_0; 

              while (i < ARRAY_LENGTH & millis() - t_0 < 5000) {
                if(distanceSensor1.checkForDataReady())
                {
                  distance1 =  distanceSensor1.getDistance(); //Get the result of the measurement from the sensor

                  distanceSensor1.stopRanging();
                  distanceSensor1.clearInterrupt();
                  distanceSensor1.startRanging();

                  i++;
                }
                // Start of Control Loop Calculations
                distance_data1[i] = distance1;
                current_tist = distance1;
                old_error = error1;
                error1 = current_tist - target_dist; //target_dist - current_tist;
                //Proportional Control
                dt = millis() - prev_time;
                prev_time = millis();
                time_data[i] = prev_time;
                sumError = sumError + (error1*dt/1000);
                if (sumError>100) {
                  sumError = 100;
                } else if (sumError < -150) {
                  sumError = -150;
                }
                Pterm = Kp * error1;
                Iterm = Ki * sumError;
                // Dterm = 0;
                Dterm = Kd * (error1 - old_error)/dt;
                pwm = Pterm + Iterm + Dterm;

                // Control Signal Saturation                  
                if(pwm > maxSpeed) pwm = maxSpeed;
                if(pwm < -maxSpeed) pwm = -maxSpeed;
                
                pwm_data[i] = pwm;
                if(pwm < 0) { //previous code: pwm > 0
                  analogWrite(MotorRightBackward,0);
                  analogWrite(MotorRightForward,pwm + 5);
                  analogWrite(MotorLeftForward,pwm*2.0 + 12);
                  analogWrite(MotorLeftBackward,0);
                }
                else if(pwm > 0) { //previous code: pwm < 0 
                  // pwm = abs(pwm);
                  analogWrite(MotorRightBackward,abs(pwm) + 5);
                  analogWrite(MotorRightForward,0);
                  analogWrite(MotorLeftForward,0);
                  analogWrite(MotorLeftBackward,abs(pwm)*2.0 + 12);
                }
                else{
                  analogWrite(MotorRightBackward,0);
                  analogWrite(MotorRightForward,0);
                  analogWrite(MotorLeftForward,0);
                  analogWrite(MotorLeftBackward,0);
                }
              }

              analogWrite(MotorRightBackward,0);
              analogWrite(MotorRightForward,0);
              analogWrite(MotorLeftForward,0);
              analogWrite(MotorLeftBackward,0);

              for (int j = 0; j < ARRAY_LENGTH; j++) {
                tx_estring_value.clear();
                tx_estring_value.append(distance_data1[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(pwm_data[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(time_data[j]);
                tx_estring_value.append("|");
                tx_estring_value.append(fast_time[j]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
                Serial.print("Overall value is:");
                Serial.println(tx_characteristic_string);

              }

              Serial.println("Sent time many times");
            break;
          }
          case CHANGE_GAIN:
          {
            float new_kp; 
            float new_ki; 
            float new_kd; 
            float new_maxSpeed
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
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void
setup()
{
    analogWrite(MotorRightBackward,0);
    analogWrite(MotorRightForward,0);
    analogWrite(MotorLeftForward,0);
    analogWrite(MotorLeftBackward,0);

    Serial.begin(115200);
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
    Serial.println("Sensor online!");
    if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
    {
      Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
      while (1)
        ;
    }
    Serial.println("Sensor online!");
    Serial.println(distanceSensor0.getI2CAddress());
    Serial.println(distanceSensor1.getI2CAddress());
    distanceSensor0.setDistanceModeLong(); //Write configuration bytes to initiate measurement
    distanceSensor1.setDistanceModeLong(); //Write configuration bytes to initiate measurement
    // distanceSensor0.setTimingBudgetInMs(33);
    // distanceSensor1.setTimingBudgetInMs(33);
// measure periodically. Intermeasurement period must be >/= timing budget.
    distanceSensor0.setIntermeasurementPeriod(40);
    distanceSensor1.setIntermeasurementPeriod(40);
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
