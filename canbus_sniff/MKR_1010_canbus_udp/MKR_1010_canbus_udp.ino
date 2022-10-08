/*

  WiFi UDP Send and Receive String

 This sketch wait an UDP packet on localPort using the WiFi module.

 When a packet is received an Acknowledge packet is sent to the client on port remotePort

 created 30 December 2012

 by dlf (Metodo2 srl)

 */

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <CAN.h>
#include <Wire.h>
#include <MKRIMU.h>


nt status = WL_IDLE_STATUS;
//#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = //SECRET_SSID;        // your network SSID (name)
char pass[] = //SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;            // your network key Index number (needed only for WEP)

unsigned int localPort = 2390;      // local port to listen on

char packetBuffer[256]; //buffer to hold incoming packet
char  ReplyBuffer[] = "acknowledged";       // a string to send back

WiFiUDP Udp;

long currentMillis = 0;
long previousMillis = 0;
int canbus_speed = 0;
int canbus_steeringangle = 0;
int canbus_rpm = 0;
int canbus_dataload[8];
bool bleClientConnected = false;
bool newCanbusData = false;
int heading_int, pitch_int, roll_int;
float gyro_x_accel = 0.0;
int lastPitchReadings[3];  // using them in fifo mode to identify the trend
int lastHeadingReadings[3];

//timekeepeing
#define seconds() (millis() / 1000)
// dummy speed;
int prevSpeed = 0;

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

  /*
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Acceleration in G's ");
  Serial.println("X\tY\tZ");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope in degrees/second ");
  Serial.println("X\tY\tZ");
  Serial.print("Euler Angles sample rate = ");
  Serial.print(IMU.eulerAnglesSampleRate());
  Serial.println(" Hz");
  Serial.print("Euler Angles in degrees ");
  Serial.println("Heading\tRoll\tPitch");
*/
  // feeding the movement trending arrays;

  for (int i = 0; i < 3; i++) {
    acceleration();
    lastPitchReadings[i] = pitch_int;
    lastHeadingReadings[i] = heading_int;
  }

  BLE.setLocalName("MYSERVICE");  // Setting a name that will appear when scanning for bluetooth devices
  BLE.setAdvertisedService(carService);

  carService.addCharacteristic(speedReading);  // add characteristics to a service
  carService.addCharacteristic(pitchReading);
  carService.addCharacteristic(headingReading);
  carService.addCharacteristic(rpmReading);
  carService.addCharacteristic(gyroXReading);

  BLE.addService(carService);  // adding the service

  speedReading.writeValue(0);  // set initial value for characteristics
  pitchReading.writeValue(0);
  headingReading.writeValue(0);
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
  }
  //currentMillis = millis();
  // ------

  if (!central) {
    // wait for the first 10 seconds to connect to ble
    // if (currentMillis - previousMillis < 10000){
    central = BLE.central();  // wait for a BLE central

    // }
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
            //headingReading.writeValue(heading_int);
            //pitchReading.writeValue(pitch_int);
            gyroXReading.writeValue(gyro_x_accel);
          }
        }
      }
    } else if (CAN.packetId() == 0x260) {
      // STEERING ANGLE e.g.  id      data length   data
      //                      0x260   8             00 00 00 00 00 FF 56 BF 19

      int i = 0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }

      if (canbus_dataload[5] > 0) {  // when 0, steering angle not moved.
        // byte 5 is taking values 255,254,253... etc when wheel moved clockwise
        // byte 5 is taking values 1, 2, 3... etc when wheel moved anticlockwise
        canbus_steeringangle = canbus_dataload[5];
        Serial.print("a");
        Serial.print(canbus_dataload[5]);
        Serial.println();
        // newCanbusData = true;
        if (central && central.connected()) {
          pitchReading.writeValue(canbus_steeringangle);
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
  // delay(50);
  /*
  float x, y, z;

  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(x, y, z);    
    Serial.print(x);
    Serial.print('\t');
    //Serial.print(y);
    //Serial.print('\t');
    //Serial.println(z);

  }
*/
  float gyro_x_accel, y, z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    if (gyro_x_accel > 1.0 || gyro_x_accel < -1) { // send to jetson. only above a threshold
      if (x > 0) {
        Serial.print("L");
      } else {
        Serial.print("R");
      }
      Serial.println(gyro_x_accel);
      //Serial.println('\t');      
    } 
    //Serial.print(y);
    //Serial.print('\t');
    //Serial.println(z);
  }
  /*
  float heading, roll, pitch;

  if (IMU.eulerAnglesAvailable()) {
    IMU.readEulerAngles(heading, roll, pitch);
    heading_int = heading;
    pitch_int = pitch;
    // move everything in the arrays one position forward.
    for (int i = 1; i > -1; i--) {
      lastPitchReadings[i + 1] = lastPitchReadings[i];
      lastHeadingReadings[i + 1] = lastHeadingReadings[i];
    }
    // add the new value at 0 position
    lastPitchReadings[0] = pitch_int;
    lastHeadingReadings[0] = heading_int;
    // calculate trend
    // NOT THE MEDIAN, maybe the mean
    //int medianPitch = QuickMedian<int>::GetMedian(lastPitchReadings, valuesInt10Length);
    //int medianHeading = QuickMedian<int>::GetMedian(lastHeadingReadings, valuesInt10Length);
    int turnIndication = 0;  // 0 = no turn, +1 = right turn, -1 = left turn
    int diffHeading = 0;
    if (lastHeadingReadings[1] >= 0 && lastHeadingReadings[1] <= 90 && lastHeadingReadings[0] <= 359 && lastHeadingReadings[0] >= 270) {
      turnIndication = -1;
    } else if (lastHeadingReadings[0] >= 0 && lastHeadingReadings[0] <= 90 && lastHeadingReadings[1] <= 359 && lastHeadingReadings[1] >= 270) {
      turnIndication = 1;
    } else if (lastHeadingReadings[0] < lastHeadingReadings[1]) {
      turnIndication = -1;
    } else if (lastHeadingReadings[0] > lastHeadingReadings[1]) {
      turnIndication = 1;
    }

    if (turnIndication < 0) {
      Serial.print("LEFT \t");
      Serial.print(x); //gyro accel
      Serial.print('\t');
      for (int i = 0; i < 3; i++) {
        Serial.print(lastHeadingReadings[i]);
        Serial.print('\t');
      }
      Serial.println();
    } else if (turnIndication > 0) {
      Serial.print("RIGHT \t");
      Serial.print(x); //gyro accel
      Serial.print('\t');
      for (int i = 0; i < 3; i++) {
        Serial.print(lastHeadingReadings[i]);
        Serial.print('\t');
      }
      Serial.println();
    }
    //Serial.print('\t');
    //Serial.println(heading_int);
    //Serial.print(roll);
    //Serial.print('\t');
    //Serial.println(pitch);
  }
  */
}
