#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include <math.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" 

#define BLE_UUID_TEST_SERVICE "02750e33-0c76-4684-8502-93f9b18da8b4"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3

SFEVL53L1X tof1;
SFEVL53L1X tof2(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X tof1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);


BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

RobotCommand robot_cmd(":|");
EString tx_estring_value;
float tx_float_value = 0.0;

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


int data_rate = 0;
int loop_count = 0;
unsigned long recording_start_time = 0;
unsigned long recording_duration = 0;

ICM_20948_I2C myICM;

enum CommandTypes {
    START_RECORD_DATA,
    STOP_RECORD_DATA,
    GET_DATA_RATE,
};

void handle_command() {
    robot_cmd.set_cmd_string(rx_characteristic_string.value(), 
                              rx_characteristic_string.valueLength());

    int cmd_type = -1;

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

void setup() {
    Wire.begin();
    Wire.setClock(115200);
    myICM.begin(Wire, 1);

    // Tof I2C setup
    pinMode(SHUTDOWN_PIN, OUTPUT);
    digitalWrite(SHUTDOWN_PIN, LOW); // Shut down ToF2
    tof1.setI2CAddress(0xf5);  // assign differen I2C address
    digitalWrite(SHUTDOWN_PIN, HIGH); // Restart ToF2
    
    if (tof1.begin() != 0) {
      Serial.println("Sensor failed to begin. ");
      while (1);
    }
    Serial.println("ToF1 online!");

    if (tof2.begin() != 0) {
      Serial.println("Sensor failed to begin. ");
      while (1);
    }
    Serial.println("ToF2 online!");

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

#define ALPHA 0.213  // Low pass filter constant for 135Hz sampling rate and 4 Hz cutoff
#define ALPHA_COMPL 0.35 

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
    else {
      Serial.println("Disconnected");
      disconnected = 1;
      connected = 0;
    }
  }
  // tof1_tof2_data_serial();
}

