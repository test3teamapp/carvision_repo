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
//#include <ArduinoLowPower.h>

BLEService carService("0000dd30-76d9-48e9-aa47-d0538d18f701");  // creating the service

BLEUnsignedIntCharacteristic speedReading("0000dd31-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);    // creating the Speed Value characteristic
BLEUnsignedIntCharacteristic rpmReading("0000dd32-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);      // creating the rpm Value characteristic
BLEFloatCharacteristic gyroXReading("0000dd33-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the heading angle Value characteristic


long canbusInterruptTimeInSeconds = 0;
long lastTimeRPMReceivedInSeconds = 0;
int canbus_speed = 0;
int canbus_rpm = 0;
int canbus_dataload[8];
// CanBus shiled Pin used to trigger a wakeup (from documentation)
const int canbusInterruptPin = 7;
BLEDevice central;
bool newCanbusData = false; // new relevant data received (speed and RPM)
bool canbusIsReceiving = false; // can bus shield is receiving from the can bus (all data)- triggered with interrupt
float gyro_x_accel = 0.0;

//timekeepeing
#define seconds() (millis() / 1000)
const unsigned long deepsleeptime = 60000; // Sleep for 4 hours before waking up to check the battery


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

  // Set interupt function that is triggered when data is received by the can bus
  CAN.onReceive(onCanBusReceive);
  // set interupt pin to wake up the main loop from deep sleep when
  // the can bus shield receives data
  //pinMode(canbusInterruptPin, INPUT);
  // Attach a wakeup interrupt on the CanBus shield interrupt pin
  // trigger function Interrupt Service Routine (ISR)
  //LowPower.attachInterruptWakeup(canbusInterruptPin, ISR, RISING);


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

  //time
  Serial.println(seconds());
}

void loop() {

  // ------ THIS LOOP WILL BE TRIGGERED BY AN INTERRUPT BY THE CAN BUS SHIELD ---//
  //Serial.println("loop is running");

  if (!central) {
    //Serial.println("connecting to central");
    //BLE.advertise();  // start advertising the service
    central = BLE.central();  // wait for a BLE central
  }else if (! central.connected()) {
    central.connect();
  }

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
    // THE ENGINE HAS STOPPED for almost 3 seconds
    newCanbusData = false;
    canbus_speed = canbus_speed + 1;
    canbus_rpm = canbus_speed + 1;
    sendBLEData(); // sent the last values. especially RPM 0
    // The last value for RPM could have been other than 0
    // so the board receiving from ble RPM values,
    // can not know if engine is on or off,
    // untill it receives an RPM = 0, so that it stops object detection

    delay(10000); // wait 10 seconds.
    // just so that BLE does not switch off before sending values
    // Set MKR 1010 to low power
    //LowPower.sleep();
    //------- FOR TESTING ---//
    lastTimeRPMReceivedInSeconds = seconds();
    // Triggers a 10000 ms sleep (the device will be woken up only by the registered wakeup sources and by internal RTC)
    // The power consumption of the chip will drop consistently
    //if (central && central.connected()){
    //  central.disconnect();
    //}
    //Serial.println("going to sleep");
    //LowPower.deepSleep(8 * 1000); // low power sleeping is ms * 4
  }
}

// Interrupt Service Routine // triggered on RISING
// this will wakeup the loop from deep sleep
void ISR()
{
  canbusIsReceiving = true;
  newCanbusData = false; // wait for the appropriate data (speed, rpm)
  canbusInterruptTimeInSeconds = seconds(); // mark when we woke up
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

// the interupt function , triggered by the canbus shiled when data arrives
void onCanBusReceive(int packetSize) {

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
