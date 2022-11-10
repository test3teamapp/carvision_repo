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
#include <MKRIMU.h>

BLEService carService("0000dd30-76d9-48e9-aa47-d0538d18f701");  // creating the service

BLEStringCharacteristic speedReading("0000dd31-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify, 5);    // creating the Speed Value characteristic
BLEStringCharacteristic rpmReading("0000dd32-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify, 10);      // creating the rpm Value characteristic
BLEStringCharacteristic gyroXReading("0000dd33-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify, 20);  // creating the heading angle Value characteristic
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
  Serial.begin(115200);//115200);  // initialize serial communication
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
  carService.addCharacteristic(statusReading);

  BLE.addService(carService);  // adding the service

  speedReading.writeValue(String(0));  // set initial value for characteristics
  rpmReading.writeValue(String(0));
  gyroXReading.writeValue(String(0.0));
  statusReading.writeValue(String("Arduino Starting.."));

  BLE.advertise();  // start advertising the service
  //Serial.println("Bluetooth device active, waiting for connections...");

  pinMode(canbusINTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(canbusINTERRUPT_PIN), CanBusInterruptFunction, CHANGE);
  //time
  //Serial.println(seconds());
}

void loop() {

  if (!central || (!central.connected())) {
    central = BLE.central();  // wait for a BLE central
  }

  // test
  /*
  //delay(1000);
  if (canbus_speed < 200) {
    canbus_speed = canbus_speed + 10;
    canbus_rpm = canbus_rpm + 10;
    Serial.println("Sending data " + String(canbus_speed));
    sendBLEData();
  } else {
    canbus_speed = 0;
    canbus_rpm = 0;
    sendBLEData();
    //delay(1000);
  }
  */
// end test

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
    } 
    canbusIsReceiving = false;
    newCanbusData = false;
  }
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
    speedReading.writeValue(String(canbus_speed));
    rpmReading.writeValue(String(canbus_rpm));
    if (canbus_speed > 0) {
      // read accelaration data
      acceleration();
      gyroXReading.writeValue(String(gyro_x_accel));
    }
  }
}

void CanBusReceive(void) {

  // do the main can-bus sniffing
  // try to parse packet

  int packetSize = CAN.parsePacket();

  if (packetSize) {
    if (!canbusIsReceiving) {
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
