/*
  Enabling BLE on MKR WiFi 1010

  This sketch controls an LED on a MKR WiFi 1010 board
  and makes a random reading of an analog pin.

  The data recorded can be accessed through Bluetooth,
  using an app such as LightBlue.

  Based on the Arduino BLE library, Battery Monitor example.

  (c) 2020 K. Söderby for Arduino
*/

#include <ArduinoBLE.h>
#include <CAN.h>
#include <Wire.h>
#include <MKRIMU.h>
//#include "QuickMedianLib.h"

BLEService carService("0000dd30-76d9-48e9-aa47-d0538d18f701");  // creating the service

BLEUnsignedIntCharacteristic speedReading("0000dd31-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);    // creating the Speed Value characteristic
BLEUnsignedIntCharacteristic rpmReading("0000dd32-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);      // creating the rpm Value characteristic
BLEFloatCharacteristic gyroXReading("0000dd33-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the heading angle Value characteristic


long currentMillis = 0;
long previousMillis = 0;
int canbus_speed = 0;
int canbus_rpm = 0;
int canbus_dataload[8];
BLEDevice central;
bool bleClientConnected = false;
bool newCanbusData = false;
float gyro_x_accel = 0.0;

//timekeepeing
#define seconds() (millis() / 1000)
// dummy speed;
int prevSpeed = 0;


void setup() {
  Wire.begin();          // i2c dfrobot accelerometer id =  0x018
  Serial.begin(9600);//115200);  // initialize serial communication
  // while (!Serial);       //starts the program if we open the serial monitor.

  // initialize BLE library
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1)
      ;
  }

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }


  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  BLE.setLocalName("MYSERVICE");  // Setting a name that will appear when scanning for bluetooth devices
  BLE.setAdvertisedService(carService);

  carService.addCharacteristic(speedReading);  // add characteristics to a service
  carService.addCharacteristic(rpmReading);
  carService.addCharacteristic(gyroXReading);

  BLE.addService(carService);  // adding the service

  speedReading.writeValue(0);  // set initial value for characteristics
  rpmReading.writeValue(0);
  gyroXReading.writeValue(0.0);

  BLE.advertise();  // start advertising the service
  Serial.println("Bluetooth device active, waiting for connections...");
  // previousMillis = millis();
  bleClientConnected = false;
  newCanbusData = false;

  //time
  Serial.println(seconds());
}

void loop() {

  // dummy speed
  // -----
  int speed = (seconds() % 10) * 10;
  if (speed != prevSpeed) {
    prevSpeed = speed;
    Serial.print("s");
    Serial.println(speed);
    Serial.print("gx");
    Serial.println(gyro_x_accel);
    speedReading.writeValue(speed);
    gyroXReading.writeValue(gyro_x_accel);
    rpmReading.writeValue(speed * 10);
    
  }
  //currentMillis = millis();
  // ------

  if (!central) {
    central = BLE.central();  // wait for a BLE central
  }
  // do the main can-bus sniffing
  // try to parse packet

  int packetSize = CAN.parsePacket();

  if (packetSize) {

    newCanbusData = false;
    // received a packet
    // Serial.print("Received ");
    // if (CAN.packetExtended()) {
    //   Serial.print("extended ");
    // }
    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      //   Serial.print("RTR ");
      // only print packet data for non-RTR packets
    } else if (CAN.packetId() == 0x610) {
      // Designation          ID      bit-start   bit-count
      // speed (high refresh)  0x0b4   40          16
      // speed (low refresh)   0x610   8           16
      // SPEED  (low frequency) e.g.  id      data length   data
      //                              0x610     8           20 00 00 64 C0 00 00 00 497

      int i = 0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }
      if (canbus_dataload[1] != 12 && canbus_dataload[2] != 80) {  // when turning on/off the lights bytes 1 and 2 are switching momentarily
        canbus_speed = canbus_dataload[2];                         // word(canbus_dataload[3], canbus_dataload[2]);// / 100;
        Serial.print("s");
        Serial.print(canbus_speed);
        Serial.println();
        if (central && central.connected()) {
          speedReading.writeValue(canbus_speed);
        }
        if (canbus_speed > 0) {
          // read accelaration data
          acceleration();

          if (central && central.connected()) {
            gyroXReading.writeValue(gyro_x_accel);
          }
        }
      }

    } else if (CAN.packetId() == 0x3B3) {
      //                            ID      bit-start   bit-count
      // engine-rev (low refresh)   0x3b3   0           16
      // RPM  e.g.  id      data length   data
      //            0x3B3    3            05 4E 28

      int i = 0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }

      canbus_rpm = canbus_dataload[0];  // word(canbus_dataload[1], canbus_dataload[0]);
      Serial.print("r");
      Serial.print(canbus_rpm);
      Serial.println();
      // newCanbusData = true;
      if (central && central.connected()) {
        rpmReading.writeValue(canbus_rpm);
      }
    }
  }

  // test
  //  read accelaration data
  acceleration();

  if (central && central.connected()) {  // if a central is connected to the peripheral
    // Serial.print("Connected to central: ");
    // Serial.println(central.address());  // print the central's BT address
    digitalWrite(LED_BUILTIN, HIGH);  // turn on the LED to indicate the connection
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // when the central disconnects, turn off the LED
    // Serial.print("Disconnected from central: ");
    // Serial.println(central.address());
  }
}

/*!
 *  @brief Print the position result.
 */
void acceleration(void) {
  delay(50);
  
  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    gyro_x_accel = x;
    //Serial.println(x);
  }
}
