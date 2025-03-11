#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"
#include <math.h>

#define BLE_UUID_TEST_SERVICE "02750e33-0c76-4684-8502-93f9b18da8b4"
#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"
#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

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
double timestamps1[NUM_SAMPLES];
double imu_data1[NUM_SAMPLES][10]; // accX_LPF, accY_LPF, accZ_LPF, Pitch_acc_LPF, Roll_acc_LPF, Pitch_gyro, Roll_gyro, Yaw_gyro, Pitch_compl, Roll_compl

int imu_data1_index = 0;
bool imu_data1_full = false;

int data_length = 0;
int data_rate = 0;
int data_count = 0;
int total_data_count = 0;
bool finish = false;
bool recording = false;
unsigned long recording_start_time = 0;
unsigned long recording_duration = 0;

ICM_20948_I2C myICM;

enum CommandTypes
{
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
                data_count = 0;  // reset sensor data store count
                total_data_count = 0;  // reset total sensor data store count
                imu_data1_index = 0;  // reset array1 
                finish = false;  // sensor data finish sending
                imu_data1_full = false;  // reset array1 
                recording_start_time = millis();  // start time
                recording_duration = duration_seconds*1000;  // end time
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
            data_rate = total_data_count / recording_duration * 1000;
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

    Wire.begin();
    Wire.setClock(400000);
    myICM.begin(Wire, 1);
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

#define ALPHA 0.189  // Low pass filter constant for 135Hz sampling rate and 4 Hz cutoff

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

void loop() {
  BLEDevice central = BLE.central();  // check ble connection
  if (!central && disconnected){  // not connected
    Serial.println("Waiting connection...");
    disconnected = 0;
  }
  if (central) {  // connected
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      
      write_data();  // check for command
      read_data();

      // sensor data muniplation
      if (myICM.dataReady() && recording) {  // if sensor ready and being required 
        myICM.getAGMT();
        
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

        roll = (roll + roll_gyro) * (1 - ALPHA) + roll_acc_LPF * ALPHA;
        pitch = (pitch + pitch_gyro) * (1 - ALPHA) + pitch_acc_LPF * ALPHA;

        data_count++;
        total_data_count++;

        if (millis() - recording_start_time >= recording_duration) {
            recording = false;
            Serial.print(total_data_count);
            Serial.print(" total data points collected in ");
            Serial.print(recording_duration/1000);
            Serial.println(" second. ");
        }
      }

      // store data
      if (data_count > 0) {  // if there is still data need to be send
          if (imu_data1_full) {  // if the first array has value to send
            tx_estring_value.clear();
            tx_estring_value.append("Time:");
            tx_estring_value.append(double(timestamps1[imu_data1_index]));
            tx_estring_value.append(", Data: ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][0]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][1]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][2]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][3]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][4]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][5]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][6]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][7]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][8]));
            tx_estring_value.append(", ");
            tx_estring_value.append(double(imu_data1[imu_data1_index][9]));
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
            data_count--;  // indicate one line of data has sent
          }
          // send previous data before replacing with new data
          timestamps1[imu_data1_index] = millis();
          imu_data1[imu_data1_index][0] = accX_LPF;
          imu_data1[imu_data1_index][1] = accY_LPF;
          imu_data1[imu_data1_index][2] = accZ_LPF;
          imu_data1[imu_data1_index][3] = pitch_acc_LPF;
          imu_data1[imu_data1_index][4] = roll_acc_LPF;
          imu_data1[imu_data1_index][5] = pitch_gyro;
          imu_data1[imu_data1_index][6] = roll_gyro;
          imu_data1[imu_data1_index][7] = yaw_gyro;
          imu_data1[imu_data1_index][8] = pitch;
          imu_data1[imu_data1_index][9] = roll;
          imu_data1_index++;  // data array index move to next element
          if (imu_data1_index >= NUM_SAMPLES) {  // if last element is stored, indicate full and use array2
              imu_data1_full = true;
              imu_data1_index = 0;
          }
      }

      else if (total_data_count && finish == false) {
        Serial.print(total_data_count);
        Serial.print(" total data points sent in ");
        int end_time = (millis() - recording_start_time)/1000;
        Serial.print(end_time);
        Serial.println(" second. ");
        finish = true;
      }
      
    //   if (!recording && imu_data1_index > 0 && !finish) {
    //     for (int i = 0; i < imu_data1_index; i++) {tx_estring_value.clear();
    //         tx_estring_value.append("Time:");
    //         tx_estring_value.append(double(timestamps1[imu_data1_index]));
    //         tx_estring_value.append(", Data: ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][0]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][1]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][2]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][3]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][4]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][5]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][6]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][7]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][8]));
    //         tx_estring_value.append(", ");
    //         tx_estring_value.append(double(imu_data1[imu_data1_index][9]));
    //         tx_characteristic_string.writeValue(tx_estring_value.c_str());
    //         data_count--;
    //     }

    //     Serial.print(total_data_count);
    //     Serial.print(" total data points sent in ");
    //     int end_time = (millis() - recording_start_time)/1000;
    //     Serial.print(end_time);
    //     Serial.println(" second. ");
    //     finish = true;
    // }

    }
    Serial.println("Disconnected");
    disconnected = 1;
  }
}
