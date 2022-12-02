#include <SPI.h>
#include <CAN.h>
#include <MKRIMU.h>
#include <WiFiNINA.h>  // use this for MKR1010 or Nano 33 IoT
//#include <WiFi101.h>  // use this for MKR1000
#include <WiFiUdp.h>

// put your network SSID and password in
// a tab called arduino_secrets.h:
#include "arduino_secrets.h"

WiFiUDP Udp;  // instance of UDP library
// the address and port of the server
IPAddress broadcastAddress;  // broadcast
const int port = 20001;      // port on which this client sends and receives
char packetBuffer[256];      //buffer to hold incoming packet

long lastTimeRPMReceivedInSeconds = 0;
long lastTimeWifiConnectedInSeconds = 0;
long lastTimeTriedToConnecteToWifiSeconds = 0;
int canbus_speed = 0;
int prev_canbus_speed = 0;
int canbus_rpm = 0;
int canbus_steeringangle = 0;
int prev_canbus_steeringangle = 0;
float gyro_x_accel = 0.0;
int canbus_dataload[8];
bool CANBUS_IS_RECEIVING = false;  // can bus shield is receiving from the can bus (all data)- triggered with interrupt
bool WIFI_IS_CONNECTED = false;
bool JETSON_IS_ON = false;
String dataStr = "";  //String data for UDP msg
//PINS
// DO NOT USE PIN 6 OR 11
// PIN 6 IS CONNECTED TO THE NUILD-IN LED, THAT WE TURN ON WHEN SWITCHING ON WIFI
// When using 11 it stops sending UDP msgs.
int powerONJetsonRelay_PIN = 5;  // when engine is ON, set to high.
int canbusINTERRUPT_PIN = 7;     // check mkr canbus shield documentation // NOT USED IN THIS SKETCH

//timekeepeing
#define seconds() (millis() / 1000)

void setup() {

  //Serial.begin(115200);  // initialize serial communication

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    while (1)
      ;
  }

  // DO NOT USE ACCELEREROMETER FOR STERRING ANGLE CALCULATION
  // USE STERRING ANGLE FROM CAN BUS --->> see further down

  //  if (!IMU.begin()) {
  //    while (1)
  //      ;
  //  }

  // pinMode(canbusINTERRUPT_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(canbusINTERRUPT_PIN), CanBusInterruptFunction, CHANGE);
  pinMode(powerONJetsonRelay_PIN, OUTPUT);
  digitalWrite(powerONJetsonRelay_PIN, 0);

  CANBUS_IS_RECEIVING = false;
  WIFI_IS_CONNECTED = false;
  JETSON_IS_ON = false;
  lastTimeRPMReceivedInSeconds = 0;
  lastTimeWifiConnectedInSeconds = 0;
}

void loop() {
  // put this in the loop, end not in setup
  // so that we reconnect
  // But use local bool instead of contiuesly using WIFI.Status()
  if (!WIFI_IS_CONNECTED) {
    connectToNetwork();
  }

  //udpReceive();

  // do the main can-bus sniffing
  // try to parse packet

  int packetSize = CAN.parsePacket();

  if (packetSize) {

    if (!CANBUS_IS_RECEIVING) {
      sendUDPData(String("cs:CanBus is Active"));
    }
    CANBUS_IS_RECEIVING = true;
    // received a packet
    //if (CAN.packetExtended()) {
    //
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
        if (canbus_speed != prev_canbus_speed) {                   // do not send if the same// do not waste resources
          dataStr = String("speed:" + String(canbus_speed));
          sendUDPData(dataStr);
        }
        prev_canbus_speed = canbus_speed;
        // DO NOT USE ACCELEREROMETER FOR STERRING ANGLE CALCULATION
        // USE STERRING ANGLE FROM CAN BUS --->> see further down
        //        if (canbus_speed > 0) {  // we care about accel only when the car is moving
        //          // read accelaration data
        //          acceleration();
        //          dataStr = String("gx:" + String(gyro_x_accel));
        //          sendUDPData(dataStr);
        //        }
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
      // do not send RPMs. We only care about RPM if they are > 0, or we are not receiving them any more
      // so we know engine is off
      //dataStr = String("rpm:" + String(canbus_rpm));
      //sendUDPData(dataStr);
      if (!JETSON_IS_ON) {
        if (canbus_rpm > 0) {
          digitalWrite(powerONJetsonRelay_PIN, 1);  // switch on power for jetson
          JETSON_IS_ON = true;
          sendUDPData(String("cs:Engine is ON"));
        }
      }
      // RPM messages come many times within a second
      // when engine is turned off, they stop comming
      // with engine but power on, other messages may arrive
      // So we use the RPM messages as indication of car engine running
      lastTimeRPMReceivedInSeconds = seconds();

    }  else if (CAN.packetId() == 0x260) {
      // STEERING ANGLE e.g.  id      data length   data
      //                      0x260   8             00 00 00 00 00 FF 56 BF 19

      int i = 0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }
      canbus_steeringangle = canbus_dataload[5];
      if (canbus_steeringangle > 0) {  // when 0, steering angle not moved.
        // byte 5 is taking values 255,254,253... etc when wheel moved clockwise
        // byte 5 is taking values 1, 2, 3... etc when wheel moved anticlockwise
        if (canbus_steeringangle != prev_canbus_steeringangle) {                   // do not send if the same// do not waste resources

          if (canbus_steeringangle > 1 && canbus_steeringangle < 180) {
            if (canbus_speed > 0) {  // we care about steering only when the car is moving
              dataStr = String("gx:" + String(-1 * canbus_steeringangle));
              sendUDPData(dataStr);
            }
          } else if (canbus_steeringangle < 255 && canbus_steeringangle > 180) {
            if (canbus_speed > 0) {  // we care about steering only when the car is moving
              dataStr = String("gx:" + String(256 - canbus_steeringangle));
              sendUDPData(dataStr);
            }
          }
        }
      }
      prev_canbus_steeringangle = canbus_steeringangle;
    }
  }

  if (JETSON_IS_ON) {
    if (seconds() - lastTimeRPMReceivedInSeconds > 20) {
      // THE ENGINE HAS STOPPED for almost 20 seconds
      if (CANBUS_IS_RECEIVING) {  // if we were receiving data until this point
        canbus_rpm = 0;
        dataStr = String("rpm:" + String(canbus_rpm));
        sendUDPData(dataStr);
        sendUDPData(String("cs:Engine is OFF"));
        digitalWrite(powerONJetsonRelay_PIN, 0);  // power off jetson
        JETSON_IS_ON = false;
        CANBUS_IS_RECEIVING = false;  // reset this. we will need to receive new CAN BUS data to be set to true again
      }
    }
  }

  if (seconds() - lastTimeWifiConnectedInSeconds > 60) { // check network connection every 1 minute
    //Serial.println("1 minute since last connection / check for connection ");
    lastTimeWifiConnectedInSeconds = seconds();
    // check if wifi is down using the actual WiFI.Status() method and not our boolean
    if (WiFi.status() != WL_CONNECTED) {
      //Serial.println("wifi is disconnected  ");
      WIFI_IS_CONNECTED = false;
      digitalWrite(LED_BUILTIN, LOW);
      connectToNetwork();
    }
  }
}


// not used currently
void udpReceive() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    //Serial.println("Contents:");
    //Serial.println(packetBuffer);
  }
}

void connectToNetwork() {
  // try to connect to the network:

  if (seconds() - lastTimeTriedToConnecteToWifiSeconds < 5) return; // do not retry connecting within 5 seconds.

  lastTimeTriedToConnecteToWifiSeconds = seconds();

  for (byte networkCounter = 0; networkCounter < sizeof(SSIDs) / sizeof(SSIDs[0]); networkCounter++) {
    digitalWrite(LED_BUILTIN, LOW);
    // Connect to WPA/WPA2 network:
    if (WiFi.begin(SSIDs[networkCounter], WiFiPasswords[networkCounter]) == WL_CONNECTED) {
      WIFI_IS_CONNECTED = true;
      lastTimeWifiConnectedInSeconds = seconds();
      break;  //connected to network
    }
  }
  if (WIFI_IS_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    IPAddress ip = WiFi.localIP();
    // get broadcast ip
    broadcastAddress = getBroadcastIP();
    Udp.begin(port);
    sendUDPData(String("cs:Arduino Connected to WIfi"));
  }
}

IPAddress getBroadcastIP() {
  IPAddress broadcastIp = WiFi.localIP();
  broadcastIp[3] = 255;
  return broadcastIp;
}

// DO NOT USE. CHANGES VALUE TO OFTEN AND DRAINS THE PROCESSING POWER
// AND IF SENDING ANYTHING OVER USD, WE SATURATE THE BUFFER

// void CanBusInterruptFunction() {
//   int val = 0;
//   val = digitalRead(canbusINTERRUPT_PIN);
//   sendUDPData(String("cs:CanBus pin value = " + String(val)));

// }

void sendUDPData(String str) {
  if (WIFI_IS_CONNECTED) {
    // start a new packet:
    Udp.beginPacket(broadcastAddress, port);
    Udp.println(str);  // add payload to it
    Udp.endPacket();   // finish and send packet
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
