#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include <math.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" 
#include <BasicLinearAlgebra.h>    //Use this library to work with matrices
using namespace BLA;               //To declare a matrix

#define BLE_UUID_TEST_SERVICE "02750e33-0c76-4684-8502-93f9b18da8b4"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 4

SFEVL53L1X tof1;
SFEVL53L1X tof2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);


BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

RobotCommand robot_cmd(":|");
EString tx_estring_value;
float tx_float_value = 0.0;

int cmd_type;
long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long timestamps_data;

#define NUM_SAMPLES 2500
double timestamps[NUM_SAMPLES];
double imu_data[NUM_SAMPLES][10]; // accX_LPF, accY_LPF, accZ_LPF, Pitch_acc_LPF, Roll_acc_LPF, Pitch_gyro, Roll_gyro, Yaw_gyro, Pitch_compl, Roll_compl
int imu_data_index = 0;
bool imu_data_full = false;
int imu_data_count = 0;
int total_imu_data_count = 0;
bool imu_finish = false;
double tof_data[NUM_SAMPLES][2]; // tof1 & tof2 distance
int tof_data_index = 0;
bool tof_data_full = false;
int tof_data_count = 0;
int total_tof_data_count = 0;
bool tof_finish = false;
bool recording = false;
bool IMUDataReady = true;
bool tofDataReady = true;

double pid_data[NUM_SAMPLES][6];  // time, tof, speed, p, i, d
float Kp;
float Ki;
float Kd;
float target_tof;
float orien_bias;
bool pidDataReady;
bool pid_data_full;
bool pid_finish;
int pid_data_index;
int pid_data_count;
int total_pid_data_count;

double pid_orien_data[NUM_SAMPLES][7];  // time, angle, speed, target_angle, err, p, i, d
float Kp_orien;
float Ki_orien;
float Kd_orien;
float target_angle;
bool pid_orienDataReady;
bool pid_orien_data_full;
bool pid_orien_finish;
int pid_orien_data_index;
int pid_orien_data_count;
int total_pid_orien_data_count;
int turn_speed;
int err_limit = 500;
float turn_dif = 1.6;
float curr_angle;
float gyr;
bool first_call;


double kf_data[NUM_SAMPLES][6];  // time, tof, speed, p, i, d
bool kfDataReady;
bool kf_data_full;
bool kf_finish;
int kf_data_index;
int kf_data_count;
int total_kf_data_count;

double flip_data[NUM_SAMPLES][4];  // time, tof, speed, kf
bool flipDataReady;
bool flip_data_full;
bool flip_finish;
int flip_data_index;
int flip_data_count;
int total_flip_data_count;
int flip_state;

int data_rate = 0;
int loop_count = 0;
unsigned long recording_start_time = 0;
unsigned long recording_duration = 0;


// IMU&ToF variable
#define ALPHA 0.213  // Low pass filter constant for 135Hz sampling rate and 4 Hz cutoff
#define ALPHA_COMPL 0.8 
float pitch = 0.0;
float roll = 0.0;
float pitch_acc = 0.0;
float roll_acc = 0.0;
float pitch_acc_LPF = 0.0;
float roll_acc_LPF = 0.0;
float prev_time = 0.0;
float pitch_gyro = 0.0;
float roll_gyro = 0.0;
float yaw_gyro = 0.0;
float accX_LPF = 0.0;  
float accY_LPF = 0.0;
float accZ_LPF = 0.0;
int disconnected = 1;
int connected = 0;
int distance1;
int distance2;

//PID variable
#define AB1IN_LEFT 3
#define AB2IN_LEFT 14
#define AB1IN_RIGHT 16
#define AB2IN_RIGHT 15

float pid_speed;
float pid_distance;    
float err;
float integral_err;
float err_d;

float pid_orien_speed;
float err_orien;
float prev_err_orien;
float integral_err_orien;
float err_d_orien;
float pid_turn_diff;

float current_time;
float start_time;
float stop_time;
float prev_err;
float dt;
float speed_control;
float max_speed = 50;
float min_speed = 40;
float max_turn_speed = 110;
float min_turn_speed = 100;
float max_flip_speed = 80;

// KF Variables 
float kf_speed;
float kf_distance;    
float d = 0.0013333;  // drag
float m = 0.0005791; // momentum
float kf;
// A, B, C matrices
Matrix<2,2> A_mat = { 0, 1,
                      0, -d/m };
Matrix<2,1> B_mat = { 0, 
                      1/m };
Matrix<1,2> C_mat = { -1, 0 };

// Discretize A & B
float delta_t = 0.02;
Matrix<2,2> I_mat = { 1, 0,
                      0, 1      };
Matrix<2,2> A_d   = { 1, 0.1,
                      0, 0.76974149 };
Matrix<2,1> B_d   = { 0,
                      172.69388197   };

// Process and measurement noise
Matrix<2,2> sig_u = { 4000, 0,
                      0, 4000 };
Matrix<1,1> sig_z = { 100 };

// Initial states
Matrix<2,2> sig   = { 5^2, 0,
                      0, 5^2 }; // initial state covariance
Matrix<2,1> x = { -1725, 
                      0      }; // initial state mean


// Stunt
float flip_speed;
float flip_distance; 
float flip_time;

// Map
float map_speed;
float map_move_time;
float map_wait_time;
float map_time_count = 0;
float map_time;
float map_tof1;
float map_tof2;
double map_data[NUM_SAMPLES][2];  // time, tof2
bool mapDataReady;
bool map_data_full;
bool map_finish;
int map_data_index;
int map_data_count;
int total_map_data_count;
int map_state;
float map_turn_diff;
float map_previous;
float angle_count = 0;
float time_gap = 0;
float gap = 500;

// Localization
float local_speed;
float local_move_time;
float local_wait_time;
float local_time_count = 0;
float local_time;
float local_tof2;
double local_data[18];  // tof2
double local_tof; 
int local_tof_index; 
int local_data_index;
float local_turn_diff;
float local_previous;
float target_angle_dif;

// Navigation
float navig_start_x;
float navig_start_y;
float navig_start_d;
float navig_goal_x;
float navig_goal_y;
float navig_speed;
float turn_diff;
float turn_max_count;
float move_max_count;
float angle_convert;

ICM_20948_I2C myICM;

enum CommandTypes {
    START_RECORD_DATA,
    STOP_RECORD_DATA,
    GET_DATA_RATE,
    PID_POSITION_CONTROL,
    PID_ORIEN_CONTROL,
    TURN_LEFT,
    TURN_RIGHT,
    STOP,
    KF_DATA,
    FLIP,
    MAP,
    LOCALIZATION,
    NAVIGATION,
};

void handle_command() {
    robot_cmd.set_cmd_string(rx_characteristic_string.value(), 
                              rx_characteristic_string.valueLength());

    cmd_type = -1;

    if (!robot_cmd.get_command_type(cmd_type)) return;

    switch (cmd_type) {
        case START_RECORD_DATA: { // Start Recording for a given period
            Serial.println("START_RECORD_DATA called");
            int duration_seconds;
            if (robot_cmd.get_next_value(duration_seconds)) {
                recording = true;  // start sensor data store
                imu_data_count = 0;  // reset sensor data store count
                total_imu_data_count = 0;  // reset total sensor data store count
                imu_data_index = 0;  // reset array1 
                imu_finish = false;  // sensor data imu_finish sending
                imu_data_full = false;  // reset array1 
                IMUDataReady = true;
                
                tof_data_count = 0;  // reset sensor data store count
                total_tof_data_count = 0;  // reset total sensor data store count
                tof_data_index = 0;  
                tof_finish = false;  
                tof_data_full = false; 
                tofDataReady = true;

                recording_start_time = millis();  // start time
                recording_duration = duration_seconds*1000;  // end time
                loop_count = 0;  // reset loop count
                Serial.print("Recording Started for ");
                Serial.print(duration_seconds);
                Serial.println(" seconds");
              }
            break;
          }
        case STOP_RECORD_DATA: { // Stop Recording
            Serial.println("STOP_RECORD_DATA called");
            recording = false;
            Serial.println("Recording Stopped");
            break;
          }
        case GET_DATA_RATE: {  // Send Data Rate
            tx_estring_value.clear();
            tx_estring_value.append("Data per second: ");
            data_rate = total_imu_data_count / recording_duration * 1000;
            tx_estring_value.append(data_rate);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            Serial.print("Data rate sent: ");
            Serial.println(data_rate);
            break;
          }
        case PID_POSITION_CONTROL: {
            Serial.println("PID_POSITION_CONTROL called");
            // initialization for Kp, Ki, Kd, Target distance
            int duration_seconds;
            int success;
            success = robot_cmd.get_next_value(duration_seconds);
            if (!success) return;
            success = robot_cmd.get_next_value(Kp);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd);
            if (!success) return;
            success = robot_cmd.get_next_value(target_tof);
            if (!success) return;
            current_time = millis();
            recording = true;  // start sensor data store
            pid_data_count = 0;  // reset sensor data store count
            total_pid_data_count = 0;  // reset total sensor data store count
            pid_data_index = 0;  // reset array1 
            pid_finish = false;  // sensor data imu_finish sending
            pid_data_full = false;  // reset array1 
            pidDataReady = true;

            recording_start_time = millis();  // start time
            recording_duration = duration_seconds*1000;  // end time
            loop_count = 0;  // reset loop count
            Serial.print("Recording Started for ");
            Serial.print(duration_seconds);
            Serial.println(" seconds");
            
            break;
          }

        case PID_ORIEN_CONTROL: {
            Serial.println("PID_ORIEN_CONTROL called");
            // initialization for Kp, Ki, Kd, Target distance
            int duration_seconds;
            int success;
            success = robot_cmd.get_next_value(duration_seconds);
            if (!success) return;
            success = robot_cmd.get_next_value(Kp_orien);
            if (!success) return;
            success = robot_cmd.get_next_value(Ki_orien);
            if (!success) return;
            success = robot_cmd.get_next_value(Kd_orien);
            if (!success) return;
            success = robot_cmd.get_next_value(target_angle);
            if (!success) return;
            success = robot_cmd.get_next_value(pid_turn_diff);
            if (!success) return;
            recording = true;  // start sensor data store
            pid_orien_data_count = 0;  // reset sensor data store count
            total_pid_orien_data_count = 0;  // reset total sensor data store count
            pid_orien_data_index = 0;  // reset array1 
            pid_orien_finish = false;  // sensor data imu_finish sending
            pid_orien_data_full = false;  // reset array1 
            pid_orienDataReady = true;
            first_call = true;
            // curr_angle = 0;
            // gyr = 0;     
            pid_orien_speed = 0;
            err_orien = 0;
            prev_err_orien = 0;
            integral_err_orien = 0;
            err_d_orien = 0;

            recording_start_time = millis();  // start time
            recording_duration = duration_seconds*1000;  // end time
            loop_count = 0;  // reset loop count
            Serial.print("Recording Started for ");
            Serial.print(duration_seconds);
            Serial.println(" seconds");
            
            break;
          }
        case TURN_LEFT: {
          Serial.println("TURN_LEFT called");
          int success;
          success = robot_cmd.get_next_value(turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(turn_dif);
          if (!success) return;
          break;
          }
        case TURN_RIGHT: {
          Serial.println("TURN_RIGHT called");
          int success;
          success = robot_cmd.get_next_value(turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(turn_dif);
          if (!success) return;
          break;
          }
        case STOP: {
          stop();
          recording = false;
          break;
          }
        case KF_DATA: {
          Serial.println("KF_PID_POSITION_CONTROL called");
          int duration_seconds;
          int success;
          success = robot_cmd.get_next_value(duration_seconds);
          if (!success) return;
          success = robot_cmd.get_next_value(Kp);
          if (!success) return;
          success = robot_cmd.get_next_value(Ki);
          if (!success) return;
          success = robot_cmd.get_next_value(Kd);
          if (!success) return;
          success = robot_cmd.get_next_value(target_tof);
          if (!success) return;
          current_time = millis();
          recording = true;  // start sensor data store
          kf_data_count = 0;  // reset sensor data store count
          total_kf_data_count = 0;  // reset total sensor data store count
          kf_data_index = 0;  // reset array1 
          kf_finish = false;  // sensor data imu_finish sending
          kf_data_full = false;  // reset array1 
          kfDataReady = true;

          recording_start_time = millis();  // start time
          recording_duration = duration_seconds*1000;  // end time
          loop_count = 0;  // reset loop count
          Serial.print("Recording Started for ");
          Serial.print(duration_seconds);
          Serial.println(" seconds");
          
          break;
          }
        case FLIP: {
          Serial.println("FLIP called");
          int duration_seconds;
          int success;
          success = robot_cmd.get_next_value(duration_seconds);
          if (!success) return;
          success = robot_cmd.get_next_value(max_flip_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(stop_time);
          if (!success) return;
          success = robot_cmd.get_next_value(target_tof);
          if (!success) return;
          success = robot_cmd.get_next_value(turn_diff);
          if (!success) return;
          current_time = millis();
          recording = true;  // start sensor data store
          flip_data_count = 0;  // reset sensor data store count
          total_flip_data_count = 0;  // reset total sensor data store count
          flip_data_index = 0;  // reset array1 
          flip_finish = false;  // sensor data imu_finish sending
          flip_data_full = false;  // reset array1 
          flipDataReady = true;
          flip_state = 1;
          flip_time = 0;

          recording_start_time = millis();  // start time
          recording_duration = duration_seconds*1000;  // end time
          loop_count = 0;  // reset loop count
          start_time = millis();
          Serial.print("Recording Started for ");
          Serial.print(duration_seconds);
          Serial.println(" seconds");
          
          break;
          }
        case MAP: {
          Serial.println("MAP called");
          int duration_seconds;
          int success;
          success = robot_cmd.get_next_value(duration_seconds);
          if (!success) return;
          success = robot_cmd.get_next_value(Kp_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(Ki_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(Kd_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(pid_turn_diff);
          if (!success) return;
          success = robot_cmd.get_next_value(max_turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(min_turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(gap);
          if (!success) return;
          current_time = millis();
          recording = true;  // start sensor data store
          map_data_count = 0;  // reset sensor data store count
          total_map_data_count = 0;  // reset total sensor data store count
          map_data_index = 0;  // reset array1 
          map_finish = false;  // sensor data imu_finish sending
          map_data_full = false;  // reset array1 
          mapDataReady = true;
          map_state = 0;
          map_time = millis();
          target_angle = curr_angle;

          recording_start_time = millis();  // start time
          recording_duration = duration_seconds*1000;  // end time
          loop_count = 0;  // reset loop count
          start_time = millis();
          Serial.print("Recording Started for ");
          Serial.print(duration_seconds);
          Serial.println(" seconds");
          
          break;
          }
        case LOCALIZATION: {
          Serial.println("LOCALIZATION called");
          int duration_seconds;
          int success;
          success = robot_cmd.get_next_value(duration_seconds);
          if (!success) return;
          success = robot_cmd.get_next_value(Kp_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(Ki_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(Kd_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(pid_turn_diff);
          if (!success) return;
          success = robot_cmd.get_next_value(max_turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(min_turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(gap);
          if (!success) return;
          current_time = millis();
          recording = true;  // start sensor data store
          local_tof = 0;
          local_data[18] = {0};
          local_data_index = 0;
          local_tof_index = 0;
          target_angle = 0;
          first_call = true;
          angle_count = 0;

          recording_start_time = millis();  // start time
          recording_duration = duration_seconds*1000;  // end time
          loop_count = 0;  // reset loop count
          start_time = millis();
          Serial.print("Recording Started for ");
          Serial.print(duration_seconds);
          Serial.println(" seconds");
          
          break;
          }
        
        case NAVIGATION: {
          Serial.println("LOCALIZATION called");
          int duration_seconds;
          int success;
          success = robot_cmd.get_next_value(duration_seconds);
          if (!success) return;
          success = robot_cmd.get_next_value(Kp);
          if (!success) return;
          success = robot_cmd.get_next_value(Ki);
          if (!success) return;
          success = robot_cmd.get_next_value(Kd);
          if (!success) return;
          success = robot_cmd.get_next_value(Kp_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(Ki_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(Kd_orien);
          if (!success) return;
          success = robot_cmd.get_next_value(pid_turn_diff);
          if (!success) return;
          success = robot_cmd.get_next_value(navig_start_x);
          if (!success) return;
          success = robot_cmd.get_next_value(navig_start_y);
          if (!success) return;
          success = robot_cmd.get_next_value(navig_goal_x);
          if (!success) return;
          success = robot_cmd.get_next_value(navig_goal_y);
          if (!success) return;
          success = robot_cmd.get_next_value(navig_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(max_turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(min_turn_speed);
          if (!success) return;
          success = robot_cmd.get_next_value(turn_diff);
          if (!success) return;
          success = robot_cmd.get_next_value(turn_max_count);
          if (!success) return;
          success = robot_cmd.get_next_value(move_max_count);
          if (!success) return;
          success = robot_cmd.get_next_value(angle_convert);
          if (!success) return;
          current_time = millis();
          max_speed = navig_speed;
          min_speed = navig_speed - 10;
          first_call = true;
          recording = true;  // start sensor data store
          
          recording_start_time = millis();  // start time
          recording_duration = duration_seconds*1000;  // end time
          loop_count = 0;  // reset loop count
          start_time = millis();
          Serial.print("Recording Started for ");
          Serial.print(duration_seconds);
          Serial.println(" seconds");
          break;
        }
        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
}

void setup() {
    Serial.begin(115200);
    BLE.begin();
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);
    BLE.addService(testService);
    tx_characteristic_float.writeValue(0.0);
    tx_estring_value.clear();
    tx_estring_value.append("[->");
    tx_estring_value.append(9.0);
    tx_estring_value.append("<-]");
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());
    BLE.advertise();

    // IMU setup
    Wire.begin();
    Wire.setClock(115200);
    myICM.begin(Wire, 1);
    float sum;
    for (int i = 0; i < 1000; i++) {
      if (myICM.dataReady()){
        myICM.getAGMT();
        sum += myICM.gyrY();
      }
    }
    orien_bias = (float) (sum / 1000);
    Serial.print("bias: ");
    Serial.println(orien_bias);

    // Tof I2C setup
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, LOW); // Shut down ToF2
    tof1.setI2CAddress(0xf5);  // assign differen I2C address
    digitalWrite(SHUTDOWN_PIN, HIGH); // Restart ToF2
    if (tof2.begin() != 0) {
      Serial.println("Sensor 2 failed to begin. ");
    }
    if (tof1.begin() != 0) {
      Serial.println("Sensor 1 failed to begin. ");
    }
    tof1.setDistanceModeLong();
    tof2.setDistanceModeLong();
}

void
write_data() {
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
read_data(){
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}


// IMU & ToF
void sensor_data_store() {
  if (recording) {
    // IMU
    // Gyroscope
    float current_time = millis();
    float dt = (micros() - prev_time) / 1000000.0;  // Convert ms to seconds
    prev_time = micros();
    
    pitch_gyro = pitch_gyro - myICM.gyrY() * dt;
    roll_gyro = roll_gyro + myICM.gyrX() * dt;
    yaw_gyro = yaw_gyro - myICM.gyrZ() * dt;

    // Accelerameter
    accX_LPF = ALPHA * myICM.accX() + (1 - ALPHA) * accX_LPF;
    accY_LPF = ALPHA * myICM.accY() + (1 - ALPHA) * accY_LPF;
    accZ_LPF = ALPHA * myICM.accZ() + (1 - ALPHA) * accZ_LPF;    
    
    pitch_acc = atan2(myICM.accX(), myICM.accZ())*180/M_PI;
    roll_acc = atan2(myICM.accY(), myICM.accZ())*180/M_PI;
    pitch_acc_LPF = atan2(accX_LPF, accZ_LPF)*180/M_PI;
    roll_acc_LPF = atan2(accY_LPF, accZ_LPF)*180/M_PI;

    // Complementary
    roll = (roll + roll_gyro) * (1 - ALPHA_COMPL) + roll_acc_LPF * ALPHA_COMPL;
    pitch = (pitch + pitch_gyro) * (1 - ALPHA_COMPL) + pitch_acc_LPF * ALPHA_COMPL;

    imu_data_count++;
    total_imu_data_count++;

    // ToF
    distance1 = tof1.getDistance(); //Get the result of the measurement from the sensor
    tof1.clearInterrupt();
    tof1.stopRanging();
    distance2 = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    
    tof_data_count++;
    total_tof_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("IMU #: ");
        Serial.print(total_imu_data_count);
        Serial.print("  |  ToF #: ");
        Serial.print(total_tof_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
    }
    IMUDataReady = false;
    tofDataReady = false;
  }

  // store data
  if (imu_data_count > 0) {  // if there is still data need to be send
      if (imu_data_full) {  // if the first array has value to send
        tx_estring_value.clear();
        tx_estring_value.append("Time:");
        tx_estring_value.append(double(timestamps[imu_data_index]));
        tx_estring_value.append(", IMU: ");
        tx_estring_value.append(double(imu_data[imu_data_index][8]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(imu_data[imu_data_index][9]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(imu_data[imu_data_index][7]));
        tx_estring_value.append(", ToF: ");
        tx_estring_value.append(double(tof_data[tof_data_index][0]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(tof_data[tof_data_index][1]));
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        imu_data_count--;  // indicate one line of data has sent
        tof_data_count--; 
      }
      
      timestamps[imu_data_index] = millis();
      imu_data[imu_data_index][0] = accX_LPF;
      imu_data[imu_data_index][1] = accY_LPF;
      imu_data[imu_data_index][2] = accZ_LPF;
      imu_data[imu_data_index][3] = pitch_acc_LPF;
      imu_data[imu_data_index][4] = roll_acc_LPF;
      imu_data[imu_data_index][5] = pitch_gyro;
      imu_data[imu_data_index][6] = roll_gyro;
      imu_data[imu_data_index][7] = yaw_gyro;
      imu_data[imu_data_index][8] = pitch;
      imu_data[imu_data_index][9] = roll;
      imu_data_index++;  // data array index move to next element
      IMUDataReady = true;
      
      tof_data[tof_data_index][0] = distance1;
      tof_data[tof_data_index][1] = distance2;
      tof_data_index++;  // data array index move to next element
      tofDataReady = true;

      if (imu_data_index >= NUM_SAMPLES) {  // if last element is stored, indicate full and use array2
          imu_data_full = true;
          imu_data_index = 0;
          tof_data_full = true;
          tof_data_index = 0;
      }
  }  

  else if (total_imu_data_count && imu_finish == false) {
    Serial.print("IMU #: ");
    Serial.print(total_imu_data_count);
    Serial.print("  |  ToF #: ");
    Serial.print(total_tof_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    tof_finish = true;
    imu_finish = true;
  }

  if (!recording && imu_data_index > 0 && !imu_finish) {
    for (int i = 0; i < imu_data_index; i++) {tx_estring_value.clear();
        tx_estring_value.append("Time:");
        tx_estring_value.append(double(timestamps[i]));
        tx_estring_value.append(", IMU: ");
        tx_estring_value.append(double(imu_data[i][8]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(imu_data[i][9]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(imu_data[i][7]));
        tx_estring_value.append(", ToF: ");
        tx_estring_value.append(double(tof_data[i][0]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(tof_data[i][1]));
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        imu_data_count--;
    }

    Serial.print("IMU #: ");
    Serial.print(total_imu_data_count);
    Serial.print("  |  ToF #: ");
    Serial.print(total_tof_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    tof_finish = true;
    imu_finish = true;
  }
}
void tof1_tof2_data_serial() {
  // Serial print tof1 and tof2
  tof1.startRanging(); //initiate measurement
  while (!tof1.checkForDataReady()){
    delay(1);
  }
  tof2.startRanging(); //initiate measurement
  while (!tof2.checkForDataReady()){
    delay(1);
  }
  int distance1 = tof1.getDistance(); //Get the result 
  int distance2 = tof2.getDistance();
  tof1.clearInterrupt();
  tof1.stopRanging();
  tof2.clearInterrupt();
  tof2.stopRanging();

  Serial.print("ToF1: ");
  Serial.print(distance1);
  Serial.print("  |  ");

  Serial.print("ToF2: ");
  Serial.print(distance2);

  Serial.println("");
}
void imu_tof_record() {
  // check for sensor data
  tof1.startRanging(); 
  tof2.startRanging(); 
  if (myICM.dataReady() && recording && IMUDataReady && tofDataReady) {  
    myICM.getAGMT();
    IMUDataReady = false;
    tofDataReady = false;
  }
  else if (!IMUDataReady && !tofDataReady){
    sensor_data_store();
  }
}


// PID position
float pid_position_tof (float Kp, float Ki, float Kd, float current_pos, float target_pos) {
  current_time = millis();
  dt = current_time - prev_time; // in ms
  prev_time = current_time;
  err = current_pos - target_pos;
  err_d = (err - prev_err) / dt;
  
  // Wind-up protection
  if (abs(err)<0.01) {
    integral_err = 0;
  }
  else {
    integral_err = integral_err + (err * dt);
  }
  if (integral_err > err_limit) {
    integral_err = err_limit;
  }
  else if (integral_err < -err_limit) {
    integral_err = -err_limit;
  }
  
  // Calculate speed control signal
  speed_control = (int)(Kp * err + Ki * integral_err + Kd * err_d);
  if (speed_control > 0){
    if (speed_control > max_speed) {
      speed_control = max_speed;
    }
    else if (speed_control < min_speed && err > 500) {
      speed_control = min_speed;
    }
  }
  else {
    if (speed_control < (-1 * max_speed)) {
      speed_control = -1 * max_speed;
    }
    else if (speed_control > (-1*min_speed) && err < -500) {
      speed_control = -1 * min_speed;
    }
  }
  if (speed_control > 0) {
    foward(speed_control);
  }
  else if (speed_control < 0) {
    backward(-1 * speed_control);
  }
  prev_err = err;
  return speed_control;
}
void foward(int speed) {
  analogWrite(AB1IN_LEFT,speed); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,speed*turn_diff); 
  analogWrite(AB2IN_RIGHT,0);
}
void backward(int speed) {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,speed);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,speed*turn_diff);
}
void stop() {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,0);
}
void pid_data_store() {
  if (recording) {
    // ToF
    pid_distance = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    
    pid_speed = pid_position_tof(Kp, Ki, Kd, pid_distance, target_tof);
    
    pid_data_count++;
    total_pid_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("PID #: ");
        Serial.print(total_pid_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    pidDataReady = false;
  }

  // store data
  if (pid_data_count > 0) {  // if there is still data need to be send
      if (pid_data_full) {
        tx_estring_value.clear();
        tx_estring_value.append("PID: ");
        tx_estring_value.append(double(pid_data[pid_data_index][0]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_data[pid_data_index][1]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_data[pid_data_index][2]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_data[pid_data_index][3]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_data[pid_data_index][4]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_data[pid_data_index][5]));
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        pid_data_count--;
      }

      pid_data[pid_data_index][0] = millis();  // time
      pid_data[pid_data_index][1] = pid_distance;  // tof
      pid_data[pid_data_index][2] = pid_speed;  // speed
      pid_data[pid_data_index][3] = Kp * err;
      pid_data[pid_data_index][4] = Ki * integral_err;
      pid_data[pid_data_index][5] = Kd * err_d;
      pid_data_index++;  // data array index move to next element
      pidDataReady = true;

      if (pid_data_index >= NUM_SAMPLES) {  // if last element is stored, indicate full and use array2
          pid_data_full = true;
          pid_data_index = 0;
      }
  }  

  else if (total_pid_data_count && pid_finish == false) {
    Serial.print("pid #: ");
    Serial.print(total_pid_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    pid_finish = true;
  }

  if (!recording && pid_data_index > 0 && !pid_finish) {
    for (int i = 0; i < pid_data_index; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("PID: ");
      tx_estring_value.append(double(pid_data[i][0]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_data[i][1]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_data[i][2]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_data[i][3]));
      tx_estring_value.append(", ToF: ");
      tx_estring_value.append(double(pid_data[i][4]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_data[i][5]));
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      pid_data_count--;
    }

    Serial.print("pid #: ");
    Serial.print(total_pid_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    pid_finish = true;
  }
}
void pid_data_not_store() {
  if (recording) {
    // ToF
    pid_distance = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    
    pid_speed = pid_position_tof(Kp, Ki, Kd, pid_distance, target_tof);
    Serial.print(pid_distance);
    Serial.print("  |  ");
    Serial.println(pid_speed);

    
    pid_data_count++;
    total_pid_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("PID #: ");
        Serial.print(total_pid_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    pidDataReady = false;
  }
}
void pid_record() {
  tof2.startRanging(); 
  if (recording && pidDataReady) {  
    pidDataReady = false;
  }
  else if (!pidDataReady){
    pid_data_store();
  }
}


// PID orientation
void reset_orientation() {
  myICM.getAGMT(); // Get initial sensor data
  orien_bias = myICM.gyrZ(); // Assume robot is stationary; use this as gyro bias
  curr_angle = 0.0;
  integral_err_orien = 0.0;
  prev_err_orien = 0.0;
  prev_time = millis(); // Reset timing
}
float pid_orient_imu() {
  myICM.getAGMT();
  gyr = myICM.gyrZ();
  current_time = millis();

  dt = (current_time - prev_time); // in ms
  curr_angle = curr_angle + (gyr - orien_bias) * dt/1000;
  err_orien = curr_angle - target_angle;
  // Serial.println(err_orien)
  err_d_orien = (err_orien - prev_err_orien) / dt;
  integral_err_orien = integral_err_orien + (err_orien * dt);
  // Wind-up protection
  if (integral_err_orien > 500) {
    integral_err_orien = 500;
  }
  else if (integral_err_orien < -500) {
    integral_err_orien = -500;
  }
  // Calculate PWM signal
  pid_orien_speed = Kp_orien * err_orien + Ki_orien * integral_err_orien + Kd_orien * err_d_orien;
  if (pid_orien_speed > 0) {
    if (pid_orien_speed > max_turn_speed) {
      pid_orien_speed = max_turn_speed;
    }
    else if (pid_orien_speed < min_turn_speed && err_orien > 2) {
      pid_orien_speed = min_turn_speed;
    }
    cw(pid_orien_speed, pid_turn_diff); 
  }
  else {
    if (pid_orien_speed < (-1 * max_turn_speed)) {
      pid_orien_speed = -1 * max_turn_speed;
    }
    else if (pid_orien_speed > (-1 * min_turn_speed) && err_orien < -2) {
      pid_orien_speed = -1 * (min_turn_speed);
    }
    ccw(-1 * pid_orien_speed, pid_turn_diff);
  }
  prev_time = millis();
  prev_err_orien = err_orien;
  return pid_orien_speed;
}
void cw(int speed, float dif) {
  analogWrite(AB1IN_LEFT,speed); 
  analogWrite(AB2IN_LEFT,0);
  analogWrite(AB1IN_RIGHT,0); 
  analogWrite(AB2IN_RIGHT,speed*dif);
}
void ccw(int speed, float dif) {
  analogWrite(AB1IN_LEFT,0); 
  analogWrite(AB2IN_LEFT,speed);
  analogWrite(AB1IN_RIGHT,speed*dif); 
  analogWrite(AB2IN_RIGHT,0);
}
void pid_orien_data_store() {
  if (recording) {
    // IMU
    pid_orien_speed = pid_orient_imu();
    
    pid_orien_data_count++;
    total_pid_orien_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("PID #: ");
        Serial.print(total_pid_orien_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    pid_orienDataReady = false;
  }

  // store data
  if (pid_orien_data_count > 0) {  // if there is still data need to be send
      if (pid_orien_data_full) {
        tx_estring_value.clear();
        tx_estring_value.append("PID: ");
        tx_estring_value.append(double(pid_orien_data[pid_orien_data_index][0]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_orien_data[pid_orien_data_index][1]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_orien_data[pid_orien_data_index][2]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_orien_data[pid_orien_data_index][3]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_orien_data[pid_orien_data_index][4]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_orien_data[pid_orien_data_index][5]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(pid_orien_data[pid_orien_data_index][6]));
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        pid_orien_data_count--;
      }

      pid_orien_data[pid_orien_data_index][0] = millis();  // time
      pid_orien_data[pid_orien_data_index][1] = curr_angle;  // tof
      pid_orien_data[pid_orien_data_index][2] = pid_orien_speed;  // speed
      pid_orien_data[pid_orien_data_index][3] = err_orien;  // speed
      pid_orien_data[pid_orien_data_index][4] = Kp_orien * err_orien;
      pid_orien_data[pid_orien_data_index][5] = Ki_orien * integral_err_orien;
      pid_orien_data[pid_orien_data_index][6] = Kd_orien * err_d_orien;
      pid_orien_data_index++;  // data array index move to next element
      pid_orienDataReady = true;

      if (pid_orien_data_index >= NUM_SAMPLES) {  // if last element is stored, indicate full and use array2
          pid_orien_data_full = true;
          pid_orien_data_index = 0;
      }
  }  

  else if (total_pid_orien_data_count && pid_orien_finish == false) {
    Serial.print("pid #: ");
    Serial.print(total_pid_orien_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    pid_orien_finish = true;
  }

  if (!recording && pid_orien_data_index > 0 && !pid_orien_finish) {
    for (int i = 0; i < pid_orien_data_index; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("PID: ");
      tx_estring_value.append(double(pid_orien_data[i][0]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_orien_data[i][1]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_orien_data[i][2]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_orien_data[i][3]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_orien_data[i][4]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_orien_data[i][5]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(pid_orien_data[i][6]));
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      pid_orien_data_count--;
    }

    Serial.print("pid #: ");
    Serial.print(total_pid_orien_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    pid_orien_finish = true;
  }
}
void pid_orien_not_data_store() {
  if (recording) {
    // IMU
    pid_orien_speed = pid_orient_imu();
    
    pid_orien_data_count++;
    total_pid_orien_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("PID #: ");
        Serial.print(total_pid_orien_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    pid_orienDataReady = false;
  }
}
void pid_orien_record() {
  if (myICM.dataReady() && recording && pid_orienDataReady) {  
    pid_orienDataReady = false;
  }
  else if (!pid_orienDataReady){
    pid_orien_not_data_store();
  }
}
void turn_left(int speed, float dif) {
  ccw(speed, dif);
  delay(3000);
  stop();
  delay(3000);
}
void turn_right(int speed, float dif) {
  cw(speed, dif);
  delay(3000);
  stop();
  delay(3000);
}

// KF
void KF(float u, float y) {
  Matrix<1,1> u_mat={u};
  Matrix<2,1> mu_p = A_d*x + B_d*u_mat;
  Matrix<2,2> sig_p = A_d*sig*(~A_d) + sig_u;
  Matrix<1,1> sig_m = C_mat*sig_p*(~C_mat) + sig_z;
  Invert(sig_m);
  Matrix<2,1> kf_gain = sig_p*(~C_mat)*(sig_m);
  Matrix<1,1> y_mat = { y };
  Matrix<1,1> y_m = y_mat - C_mat*mu_p;
  // Update
  x = mu_p + kf_gain*y_m;
  sig = (I_mat - kf_gain*C_mat)*sig_p;
}
float kf_position_tof (float Kp, float Ki, float Kd, float current_pos, float target_pos) {
  current_time = millis();
  dt = current_time - prev_time; // in ms
  prev_time = current_time;
  KF(speed_control / 60, -1 * current_pos);
  float estimated_pos = x(0,0); 
  err = estimated_pos - target_pos;  
  err_d = (err - prev_err) / dt;
  
  // Wind-up protection
  if (abs(err)<0.01) {
    integral_err = 0;
  }
  else {
    integral_err = integral_err + (err * dt);
  }
  if (integral_err > err_limit) {
    integral_err = err_limit;
  }
  else if (integral_err < -err_limit) {
    integral_err = -err_limit;
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
    Serial.println("foward");
    foward(speed_control);
  }
  else if (speed_control < 0) {
    Serial.println("backward");
    backward(-1 * speed_control);
  }
  prev_err = err;
  return speed_control;
}
void kf_data_store() {
  if (recording) {
    // ToF
    kf_distance = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    
    kf_speed = kf_position_tof(Kp, Ki, Kd, kf_distance, target_tof);
    // if(kf_distance<50) {
    //   stop();
    //   recording = false;
    // }
    // else {
    //   foward(60);
    // }

    kf_data_count++;
    total_kf_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("KF #: ");
        Serial.print(total_kf_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    kfDataReady = false;
  }

  // store data
  if (kf_data_count > 0) {  // if there is still data need to be send
      if (kf_data_full) {
        tx_estring_value.clear();
        tx_estring_value.append("KF: ");
        tx_estring_value.append(double(kf_data[kf_data_index][0]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(kf_data[kf_data_index][1]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(kf_data[kf_data_index][2]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(kf_data[kf_data_index][3]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(kf_data[kf_data_index][4]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(kf_data[kf_data_index][5]));
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        kf_data_count--;
      }

      kf_data[kf_data_index][0] = millis();  // time
      kf_data[kf_data_index][1] = kf_distance;  // tof
      kf_data[kf_data_index][2] = kf_speed;  // speed
      kf_data[kf_data_index][3] = Kp * err;
      kf_data[kf_data_index][4] = Ki * integral_err;
      kf_data[kf_data_index][5] = Kd * err_d;
      kf_data_index++;  // data array index move to next element
      kfDataReady = true;

      if (kf_data_index >= NUM_SAMPLES) {  // if last element is stored, indicate full and use array2
          kf_data_full = true;
          kf_data_index = 0;
      }
  }  

  else if (total_kf_data_count && kf_finish == false) {
    Serial.print("KF #: ");
    Serial.print(total_kf_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    kf_finish = true;
  }

  if (!recording && kf_data_index > 0 && !kf_finish) {
    for (int i = 0; i < kf_data_index; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("KF: ");
      tx_estring_value.append(double(kf_data[i][0]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(kf_data[i][1]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(kf_data[i][2]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(kf_data[i][3]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(kf_data[i][4]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(kf_data[i][5]));
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      kf_data_count--;
      // Serial.println(kf_data[i][1]);
    }

    Serial.print("KF #: ");
    Serial.print(total_kf_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    kf_finish = true;
  }
}
void kf_data_not_store() {
  if (recording) {
    // ToF
    kf_distance = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    
    kf_speed = kf_position_tof(Kp, Ki, Kd, kf_distance, target_tof);

    kf_data_count++;
    total_kf_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("KF #: ");
        Serial.print(total_kf_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    kfDataReady = false;
  }
}
void kf_record() {
  // check for sensor data
  tof2.startRanging(); 
  if (recording) {
    kf_data_store();
  }
}

// Stunt
float flip_position_tof (float current_pos, float target_pos) {
  current_time = millis()-start_time;
  KF(speed_control / max_flip_speed, -1 * current_pos);
  float estimated_pos = x(0,0); 
  
  if (flip_state == 1 && current_time > stop_time) {
    stop();
  }
  else if (flip_state == 1 && estimated_pos > target_pos + 100) {
    Serial.println("foward");
    speed_control = max_flip_speed;
    foward(speed_control);
    flip_state = 1;
  }
  else if (flip_state == 1 && estimated_pos < target_pos + 100) {
    Serial.println("backward");
    speed_control = -1 * max_flip_speed;
    backward(-1 * speed_control);
    flip_state = 2;
    if (flip_time == 0){
      flip_time = current_time;
    }
  }
  else if (flip_state == 2 && current_time > flip_time*2) {
    Serial.println("stop");
    speed_control = 0;
    stop();
    flip_state = 3;
  }
  
  return speed_control;
}
void flip_data_store() {
  if (recording) {
    // ToF
    flip_distance = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    
    flip_speed = flip_position_tof(flip_distance, target_tof);

    flip_data_count++;
    total_flip_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("Flip #: ");
        Serial.print(total_flip_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    flipDataReady = false;
  }

  // store data
  if (flip_data_count > 0) {  // if there is still data need to be send
      if (flip_data_full) {
        tx_estring_value.clear();
        tx_estring_value.append("Flip: ");
        tx_estring_value.append(double(flip_data[flip_data_index][0]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(flip_data[flip_data_index][1]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(flip_data[flip_data_index][2]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(flip_data[flip_data_index][3]));
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        flip_data_count--;
      }

      flip_data[flip_data_index][0] = millis()-start_time;  // time
      flip_data[flip_data_index][1] = flip_distance;  // tof
      flip_data[flip_data_index][2] = flip_speed;  // speed
      flip_data[flip_data_index][3] = flip_distance;
      flip_data_index++;  // data array index move to next element
      flipDataReady = true;

      if (flip_data_index >= NUM_SAMPLES) {  // if last element is stored, indicate full and use array2
          flip_data_full = true;
          flip_data_index = 0;
      }
  }  

  else if (total_flip_data_count && flip_finish == false) {
    Serial.print("Flip #: ");
    Serial.print(total_flip_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    flip_finish = true;
  }

  if (!recording && flip_data_index > 0 && !flip_finish) {
    for (int i = 0; i < flip_data_index; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("Flip: ");
      tx_estring_value.append(double(flip_data[i][0]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(flip_data[i][1]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(flip_data[i][2]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(flip_data[i][3]));
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      flip_data_count--;
    }

    Serial.print("Flip #: ");
    Serial.print(total_flip_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    flip_finish = true;
  }
}
void flip_data_not_store() {
  if (recording) {
    // ToF
    flip_distance = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    
    flip_speed = flip_position_tof(flip_distance, target_tof);

    flip_data_count++;
    total_flip_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("Flip #: ");
        Serial.print(total_flip_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    flipDataReady = false;
  }
}
void flip_record() {
  // check for sensor data
  tof2.startRanging(); 
  if (recording) {
    flip_data_not_store();
  }
}


// Map
void map_data_store() {
  if (recording) {
    // ToF
    map_tof2 = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();    
    map_data_count++;
    total_map_data_count++;
    if (angle_count < 50 && millis() - time_gap >= gap) {
      target_angle += 5;
      angle_count++;
      time_gap = millis();
    }
    pid_orien_speed = pid_orient_imu();

    if (angle_count >= 50 || millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("Map #: ");
        Serial.print(total_map_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    mapDataReady = false;
  }

  // store data
  if (map_data_count > 0) {  // if there is still data need to be send
      if (map_data_full) {
        tx_estring_value.clear();
        tx_estring_value.append("Map: ");
        tx_estring_value.append(double(map_data[map_data_index][0]));
        tx_estring_value.append(", ");
        tx_estring_value.append(double(map_data[map_data_index][1]));
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
        map_data_count--;
      }
      
      map_data[map_data_index][0] = millis()-start_time;  // time
      map_data[map_data_index][1] = map_tof2;  // speed
      map_data_index++;  // data array index move to next element
      mapDataReady = true;

      if (map_data_index >= NUM_SAMPLES) {  
          map_data_full = true;
          map_data_index = 0;
      }
  }  

  else if (total_map_data_count && map_finish == false) {
    Serial.print("Map #: ");
    Serial.print(total_map_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    map_finish = true;
  }

  if (!recording && map_data_index > 0 && !map_finish) {
    for (int i = 0; i < map_data_index; i++) {
      tx_estring_value.clear();
      tx_estring_value.append("Map: ");
      tx_estring_value.append(double(map_data[i][0]));
      tx_estring_value.append(", ");
      tx_estring_value.append(double(map_data[i][1]));
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
      map_data_count--;
    }

    Serial.print("Map #: ");
    Serial.print(total_map_data_count);
    Serial.print("  |  Send Time: ");
    int end_time = (millis() - recording_start_time)/1000;
    Serial.println(end_time);
    map_finish = true;
  }
}
void map_data_not_store() {
  if (recording) {
    // ToF
    // map_tof1 = tof1.getDistance(); //Get the result of the measurement from the sensor
    // tof1.clearInterrupt();
    // tof1.stopRanging();
    map_tof2  = tof2.getDistance(); //Get the result of the measurement from the sensor
    tof2.clearInterrupt();
    tof2.stopRanging();
    if (angle_count < 40 && millis() - time_gap >= 500) {
      target_angle += 5;
      angle_count++;
      time_gap = millis();
    }
    pid_orien_speed = pid_orient_imu();
    
    map_data_count++;
    total_map_data_count++;

    if (millis() - recording_start_time >= recording_duration) {
        recording = false;
        Serial.print("Map #: ");
        Serial.print(total_map_data_count);
        Serial.print("  |  Time: ");
        Serial.print(recording_duration/1000);
        Serial.print("  |  Loop: ");
        Serial.println(loop_count);
        stop();
    }
    mapDataReady = false;
  }
}
void map_record() {
  // check for sensor data
  if (recording) {
    tof2.startRanging(); 
    map_data_store();
  }
}

// Localization
void local_data_store() {
  if (recording) {
    // ToF
    local_tof += tof2.getDistance(); //Get the result of the measurement from the sensor
    local_tof_index++;
    tof2.clearInterrupt();
    tof2.stopRanging(); 
    if (first_call){
      first_call = false;
      time_gap = millis();
      reset_orientation();
    }
    pid_orien_speed = pid_orient_imu();
    if (angle_count < 20 && millis() - time_gap >= gap) {
      target_angle += 12;
      angle_count++;
      time_gap = millis();
      if (local_data_index<18){
        local_data[local_data_index] = local_tof/local_tof_index;
        local_data_index++;
        if (local_data_index==18){
          Serial.println("new data collected!");
        }
      }
      local_tof_index = 0;
      local_tof = 0;
    }

    if (angle_count >=20 || millis() - recording_start_time >= recording_duration) {
        recording = false;
        stop();
        for (int i = 0; i < 18; i++) {
          tx_estring_value.clear();
          tx_estring_value.append("Localization: ");
          tx_estring_value.append(double(local_data[i]));
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
        Serial.println("data sent!");
    }

  }
}
void local_data_not_store() {
  if (recording) {
    if (angle_count < 19 && millis() - time_gap >= gap) {
      target_angle += 11;
      angle_count++;
      time_gap = millis();
    }
    if (first_call){
      first_call = false;
      reset_orientation();
    }
    pid_orien_speed = pid_orient_imu();
    if (angle_count >= 19 || millis() - recording_start_time >= recording_duration) {
      recording = false;
      stop();
    }
  }
}
void local_record() {
  // check for sensor data
  if (recording) {
    tof2.startRanging(); 
    local_data_store();
  }
  // local_data_not_store();
}


// Navigation
void navigate_to_target() {
  if (recording){
    float dx = (navig_goal_x - navig_start_x)*305;
    float dy = (navig_goal_y - navig_start_y)*305;

    // Calculate desired angle in radian to target
    float desired_theta = atan2(dy, dx) * angle_convert/9/3.1415926*180; 

    // Adjust to [-180, 180] range 
    if (desired_theta > 100.0) {
      desired_theta -= 200.0;
    } else if (desired_theta < -100.0) {
      desired_theta += 200.0;
    }
    // Rotate to desired heading
    if (first_call){
      first_call = false;
      reset_orientation();
    }
    target_angle = desired_theta;
    int count = 0;
    while (count<turn_max_count && desired_theta!=0) {
      pid_orien_speed = pid_orient_imu();
      count += 1;
    }

    float distance2 = 0;
    for (int i = 0; i < 100; i++) {
      tof2.startRanging();
      distance2+=tof2.getDistance();
      tof2.stopRanging();
    }
    distance2 = distance2 / 100;

    // Step 2: Drive forward the distance to the target
    float movement = sqrt(dx * dx + dy * dy);
    float target_pos = distance2 - movement; // Distance in meters

    float current_pos;
    err = 0;
    err_d = 0;  
    integral_err = 0;
    prev_err = 0;
    prev_time = millis();
    while (count<move_max_count) {
      tof2.startRanging();
      current_pos=tof2.getDistance();
      pid_orien_speed = pid_position_tof(Kp, Ki, Kd, current_pos, target_pos);
      tof2.stopRanging();
      count += 1;
    }
    stop();

    // Step 3: Rotate to face 0 degrees
    count = 0;
    target_angle = 0.0;
    while (count<turn_max_count && desired_theta!=0) {
      pid_orien_speed = pid_orient_imu();
      count += 1;
    }
    stop();
    recording = false;
    delay(500);
  }
}

void loop() {
  if (recording) {
    loop_count ++;
  }
  BLEDevice central = BLE.central();  // check ble connection
  if (!central && disconnected){  // not connected
    Serial.println("Waiting connection...");
    disconnected = 0;
  }
  if (central) {  // connected
    if (!connected) {
      Serial.print("Connected to: ");
      Serial.println(central.address());
      connected = 1;
    }

    if (central.connected()) {
      write_data();  // check for command
      read_data();

      if (cmd_type == 11){
        local_record();
      }
      else if (cmd_type == 12){
        navigate_to_target();
      }
    }
    else {
      Serial.println("Disconnected");
      disconnected = 1;
      connected = 0;
    }
  }
}

