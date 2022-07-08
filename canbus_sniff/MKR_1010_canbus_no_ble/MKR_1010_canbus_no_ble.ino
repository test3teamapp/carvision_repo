/*

*/

#include <CAN.h>
#include <Wire.h>
#include <MKRIMU.h>

long currentMillis = 0;
long previousMillis = 0;
int canbus_speed = 0;
int canbus_steeringangle = 0;
int canbus_rpm = 0;
int canbus_dataload[8];
float gyro_x_accel = 0.0;
float gyro_y_accel = 0.0;

//PINS USED
// CAN BUS : D3, B8, D9, D10
// IMU SHILED : D11 (SDA), D12 (SCL)
// TRIGGER PIN FOR POWERING ON JETSON USING RELAY
#define RELAY_PIN 6

//timekeepeing
#define seconds() (millis() / 1000)
// dummy speed;
int prevSpeed = 0;

bool isCarEngineRunning = false;
long secondsOfLastRPMCanMsg;

//DFRobot_LIS2DH12 LIS; // Accelerometer

void setup() {
  Wire.begin();        // i2c dfrobot accelerometer id =  0x018
  Serial.begin(9600);  //115200);  // initialize serial communication

  // RELAY for powering Jetson.
  pinMode(RELAY_PIN, OUTPUT);    //Set pin RELAY_PIN as an 'output' pin
  digitalWrite(RELAY_PIN, LOW);  // CIRCUIT IS OPEN on startup

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

  isCarEngineRunning = false;
}

void loop() {

  // do the main can-bus sniffing
  // try to parse packet

  int packetSize = CAN.parsePacket();

  if (packetSize) {

    // read accelaration data
    acceleration();

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

        // check if the engine is stopped.
        if (seconds() - secondsOfLastRPMCanMsg > 5) {
          // we haven;t received RPMs for 5 seconds. Engine must have stopped.
          // stop jetson
          isCarEngineRunning = false;
          digitalWrite(RELAY_PIN, LOW);
          canbus_rpm = 0;
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
      if (canbus_rpm > 0) {
        // close the relay circuit to power up jetson
        isCarEngineRunning = true;
        digitalWrite(RELAY_PIN, HIGH);
        secondsOfLastRPMCanMsg = seconds();
      }
    }
  }

  if (isCarEngineRunning) {
    digitalWrite(LED_BUILTIN, HIGH);  // turn on the LED to indicate the connection
  } else {
    digitalWrite(LED_BUILTIN, LOW);  // else, turn off the LED
  }
}

/*!
 *  @brief Print the position result.
 */
void acceleration(void) {
  // delay(50);

  float z;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyro_x_accel, gyro_y_accel, z);
    if (gyro_x_accel > 1.0 || gyro_x_accel < -1) {  // send to jetson. only above a threshold
      if (gyro_x_accel > 0) {
        Serial.print("L");
      } else {
        Serial.print("R");
      }
      Serial.println(gyro_x_accel);
      //Serial.println('\t');
    }

    if (gyro_y_accel > 1.0 || gyro_y_accel < -1) {  // send to jetson. only above a threshold
      if (gyro_y_accel > 0) {
        Serial.print("D");
      } else {
        Serial.print("U");
      }
      Serial.println(gyro_y_accel);
      //Serial.println('\t');
    }
    //Serial.print(y);
    //Serial.print('\t');
    //Serial.println(z);
  }
}