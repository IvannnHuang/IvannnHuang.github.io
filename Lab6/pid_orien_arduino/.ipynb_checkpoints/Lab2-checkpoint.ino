#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "02750e33-0c76-4684-8502-93f9b18da8b4"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"
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
unsigned long timestamps_data;
//////////// Global Variables ////////////
unsigned long timestamps[1000];  // Array to store timestamps
unsigned long temperature[1000];  // Array to store temperature

enum CommandTypes
{
    SEND_ACCE_DATA,
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
        
        case SEND_ACCE_DATA: {
          int index = 0;
          int count = 100;
          if (index < count){
            Serial.print(myICM.accX());  
            Serial.print("    ");  
            Serial.print(myICM.accY());  
            Serial.print("    ");
            Serial.print(myICM.accZ());  
            Serial.print("    ");
            Serial.print(int(millis());
            index++;
          }
          break;
        }

        default:
            Serial.print("Invalid Command Type: ");
            Serial.println(cmd_type);
            break;
    }
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

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

void setup()
{

  SERIAL_PORT.begin(115200);

  // BLE setup

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


  while (!SERIAL_PORT)
  {
  };

  // IMU setup

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}

#define NUM_SAMPLES 1000  // Number of samples to store

double pitch = 0.0;
double roll = 0.0;
double accelData[NUM_SAMPLES][4];
int index = 0;


// #ifdef USE_SPI
// void calculate_pitch_roll(ICM_20948_SPI *sensor)
// {
// #else
// void calculate_pitch_roll(ICM_20948_I2C *sensor)
// {
// #endif
//   pitch = atan2(sensor->accX(), sensor->accZ())*180/M_PI;
//   roll = atan2(sensor->accY(), sensor->accZ())*180/M_PI;
//   Serial.print("Pitch: ");
//   Serial.print(pitch);
//   Serial.print("  ||  ");
//   Serial.print("Roll: ");
//   Serial.println(roll);

// }

void loop()
{  
  if (myICM.dataReady())
  {
    myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                             //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
    // calculate_pitch_roll(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    // pitch = atan2(myICM.accX(), myICM.accZ())*180/M_PI;
    // roll = atan2(myICM.accY(), myICM.accZ())*180/M_PI;
    // Serial.print("Pitch: ");
    // Serial.print(pitch);
    // Serial.print("  ||  ");
    // Serial.print("Roll: ");
    // Serial.println(roll);

    if (index < NUM_SAMPLES){
      accelData[index][0] = myICM.accX();  
      accelData[index][1] = myICM.accY();  
      accelData[index][2] = myICM.accZ();  
      accelData[index][3] = int(millis());
      index++;  
    }
    else{
      Serial.print("Data array full!");
    }
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }

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

// Below here are some helper functions to print the data nicely!

// void printPaddedInt16b(int16_t val)
// {
//   if (val > 0)
//   {
//     SERIAL_PORT.print(" ");
//     if (val < 10000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (val < 1000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (val < 100)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (val < 10)
//     {
//       SERIAL_PORT.print("0");
//     }
//   }
//   else
//   {
//     SERIAL_PORT.print("-");
//     if (abs(val) < 10000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (abs(val) < 1000)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (abs(val) < 100)
//     {
//       SERIAL_PORT.print("0");
//     }
//     if (abs(val) < 10)
//     {
//       SERIAL_PORT.print("0");
//     }
//   }
//   SERIAL_PORT.print(abs(val));
// }

// void printRawAGMT(ICM_20948_AGMT_t agmt)
// {
//   SERIAL_PORT.print("RAW. Acc [ ");
//   printPaddedInt16b(agmt.acc.axes.x);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.acc.axes.y);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.acc.axes.z);
//   SERIAL_PORT.print(" ], Gyr [ ");
//   printPaddedInt16b(agmt.gyr.axes.x);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.gyr.axes.y);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.gyr.axes.z);
//   SERIAL_PORT.print(" ], Mag [ ");
//   printPaddedInt16b(agmt.mag.axes.x);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.mag.axes.y);
//   SERIAL_PORT.print(", ");
//   printPaddedInt16b(agmt.mag.axes.z);
//   SERIAL_PORT.print(" ], Tmp [ ");
//   printPaddedInt16b(agmt.tmp.val);
//   SERIAL_PORT.print(" ]");
//   SERIAL_PORT.println();
// }

// void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
// {
//   float aval = abs(val);
//   if (val < 0)
//   {
//     SERIAL_PORT.print("-");
//   }
//   else
//   {
//     SERIAL_PORT.print(" ");
//   }
//   for (uint8_t indi = 0; indi < leading; indi++)
//   {
//     uint32_t tenpow = 0;
//     if (indi < (leading - 1))
//     {
//       tenpow = 1;
//     }
//     for (uint8_t c = 0; c < (leading - 1 - indi); c++)
//     {
//       tenpow *= 10;
//     }
//     if (aval < tenpow)
//     {
//       SERIAL_PORT.print("0");
//     }
//     else
//     {
//       break;
//     }
//   }
//   if (val < 0)
//   {
//     SERIAL_PORT.print(-val, decimals);
//   }
//   else
//   {
//     SERIAL_PORT.print(val, decimals);
//   }
// }


// #ifdef USE_SPI
// void printScaledAGMT(ICM_20948_SPI *sensor)
// {
// #else
// void printScaledAGMT(ICM_20948_I2C *sensor)
// {
// #endif
//   // Read raw accelerometer data
//   float accX_RAW = sensor->accX();
//   float accY_RAW = sensor->accY();
//   float accZ_RAW = sensor->accZ();

//   // Apply low-pass filter
//   accX_LPF = applyLowPassFilter(accX_RAW, accX_LPF);
//   accY_LPF = applyLowPassFilter(accY_RAW, accY_LPF);
//   accZ_LPF = applyLowPassFilter(accZ_RAW, accZ_LPF);

//   SERIAL_PORT.print("Scaled. Acc (mg) [ ");
//   printFormattedFloat(sensor->accX(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(accY_RAW, 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(accZ_RAW, 5, 2);
//   SERIAL_PORT.print(" ], Gyr (DPS) [ ");
//   printFormattedFloat(sensor->gyrX(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->gyrY(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->gyrZ(), 5, 2);
//   SERIAL_PORT.print(" ], Mag (uT) [ ");
//   printFormattedFloat(sensor->magX(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->magY(), 5, 2);
//   SERIAL_PORT.print(", ");
//   printFormattedFloat(sensor->magZ(), 5, 2);
//   SERIAL_PORT.print(" ], Tmp (C) [ ");
//   printFormattedFloat(sensor->temp(), 5, 2);
//   SERIAL_PORT.print(" ]");
//   SERIAL_PORT.println();
// }
