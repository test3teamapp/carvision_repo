/*
  Enabling BLE on MKR WiFi 1010

  This sketch controls an LED on a MKR WiFi 1010 board
  and makes a random reading of an analog pin.

  The data recorded can be accessed through Bluetooth,
  using an app such as LightBlue.

  Based on the Arduino BLE library, Battery Monitor example.

  (c) 2020 K. Söderby for Arduino
*/

/// ------------------------ WE CAN NOT USE BOTH BLE AND WIFI AT THE SAME TIME ---------- WITH MKR1010

#include <ArduinoBLE.h>
#include <CAN.h>
#include <MKRIMU.h>
#include <WiFiNINA.h>   // use this for MKR1010 or Nano 33 IoT
//#include <WiFi101.h>  // use this for MKR1000

// put your network SSID and password in
// a tab called arduino_secrets.h:
#include "arduino_secrets.h"

BLEService carService("0000dd30-76d9-48e9-aa47-d0538d18f701");  // creating the service

BLEUnsignedIntCharacteristic speedReading("0000dd31-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);    // creating the Speed Value characteristic
BLEUnsignedIntCharacteristic rpmReading("0000dd32-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);      // creating the rpm Value characteristic
BLEFloatCharacteristic gyroXReading("0000dd33-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the heading angle Value characteristic
BLEStringCharacteristic statusReading("0000dd34-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify, 50);  // creating the status (50chars max) Value characteristic


long lastTimeRPMReceivedInSeconds = 0;
int canbus_speed = 0;
int canbus_rpm = 0;
int canbus_dataload[8];
int canbusINTERRUPT_PIN = 7; // check mkr canbus shield documentation
BLEDevice central;
bool newCanbusData = false; // new relevant data received (speed and RPM)
bool canbusIsReceiving = false; // can bus shield is receiving from the can bus (all data)- triggered with interrupt
float gyro_x_accel = 0.0;

//timekeepeing
#define seconds() (millis() / 1000)

void setup() {
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
  //Serial.println("Bluetooth device active, waiting for connections...");

  pinMode(canbusINTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(canbusINTERRUPT_PIN), CanBusInterruptFunction, CHANGE);

  connectToNetwork(); // connect to wifi if available
  //time
  //Serial.println(seconds());
}

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connectToNetwork();
  }
  if (!central || (!central.connected())) {
    central = BLE.central();  // wait for a BLE central
  }

  CanBusReceive();

  if (newCanbusData) {
    sendBLEData();
  }

  if (central && central.connected()) {  // if a central is connected to the peripheral
    //Serial.print("Connected to central: ");
    // Serial.println(central.address());  // print the central's BT address
    digitalWrite(LED_BUILTIN, HIGH);  // turn on the LED to indicate the connection
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // when the central disconnects, turn off the LED
    //Serial.print("Disconnected from central: ");
    // Serial.println(central.address());
  }


  if (seconds() - lastTimeRPMReceivedInSeconds > 10) {
    // THE ENGINE HAS STOPPED for almost 10 seconds
    if (canbusIsReceiving) { // if we were receiving data until this point
      statusReading.writeValue(String("Engine has stopped"));
      canbus_speed = 0;
      canbus_rpm = 0;
      sendBLEData(); // sent the last values. especially RPM 0
    } else {
      if (central && central.connected()) {
        statusReading.writeValue(String("Engine is off"));
      }
    }
    canbusIsReceiving = false;
    newCanbusData = false;
  }
}

void connectToNetwork() {
  // try to connect to the network:
  while ( WiFi.status() != WL_CONNECTED) {
    //digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(SECRET_SSID);       // print the network name (SSID);
    // Connect to WPA/WPA2 network:
    WiFi.begin(SECRET_SSID, SECRET_PASS);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  // print the SSID of the network you're attached to:
  if (Serial) Serial.print("Connected to: ");
  if (Serial) Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  if (Serial) Serial.print("IP Address: ");
  if (Serial) Serial.println(ip);
}

void CanBusInterruptFunction() {
  int val = 0;
  val = digitalRead(canbusINTERRUPT_PIN);
  if (central && central.connected()) {
    statusReading.writeValue(String("CanBus INT Pin status changed to " + String(val)));
  }

}

void sendBLEData() {
  if (central && central.connected()) {
    speedReading.writeValue(canbus_speed);
    rpmReading.writeValue(canbus_rpm);
    if (canbus_speed > 0) {
      // read accelaration data
      acceleration();
      gyroXReading.writeValue(gyro_x_accel);
    }
  }
}

void CanBusReceive(void) {

  // do the main can-bus sniffing
  // try to parse packet

  int packetSize = CAN.parsePacket();

  if (packetSize) {
    if (!canbusIsReceiving){
      if (central && central.connected()) {
        statusReading.writeValue(String("CanBus is Active"));
      }
    }
    canbusIsReceiving = true;
    newCanbusData = false;
    // received a packet
    Serial.print("Received ");
    Serial.println(CAN.packetId());
    //if (CAN.packetExtended()) {
    //  Serial.print("extended ");
    //}
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
        newCanbusData = true;
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
      newCanbusData = true;
      // RPM messages come many times within a second
      // when engine is turned off, they stop comming
      // with engine but power on, other messages may arrive
      // So we use the RPM messages as indication of car engine running
      lastTimeRPMReceivedInSeconds = seconds();
    }
  }
}

/*!
    @brief get the G angle of the car on the x axis
*/
void acceleration(void) {
  //delay(50);

  float x, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    // the way we install it in the car, to measure the "car's x g movement" we
    // need to track the devices' "y" axis
    gyro_x_accel = y;
  }
}
