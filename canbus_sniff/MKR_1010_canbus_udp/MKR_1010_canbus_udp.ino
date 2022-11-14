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

/*
   Messages from arduino must be Strings, ending with newline,
   and starting with the following prefixes (adding after it ":")
   e.g. speed:20
        rpm:4000
        gx:-2.5
        cs:Car engine is OFF

  public enum OutputType {
    SPEED("s"),  //calls constructor with value s
    HEADING_ANGLE("h"),  //calls constructor with value a
    RPM("r"),
    PITCH_ANGLE("p"),
    GYRO_X_ANGLE("gx"),
    GYRO_Y_ANGLE("gy)"),
    MSG("msg"),   //calls constructor with value m
    CARSTATUS("cs")
*/


#include <SPI.h>
#include <CAN.h>
#include <MKRIMU.h>
#include <WiFiNINA.h>   // use this for MKR1010 or Nano 33 IoT
//#include <WiFi101.h>  // use this for MKR1000
#include <WiFiUdp.h>

// put your network SSID and password in
// a tab called arduino_secrets.h:
#include "arduino_secrets.h"

WiFiUDP Udp;           // instance of UDP library
// the address and port of the server
IPAddress broadcastAddress; // broadcast
const int port = 20001; // port on which this client sends and receives
char packetBuffer[256]; //buffer to hold incoming packet

long lastTimeRPMReceivedInSeconds = 0;
int canbus_speed = 0;
int canbus_rpm = 0;
float gyro_x_accel = 0.0;
int canbus_dataload[8];
bool newCanbusData = false; // new relevant data received (speed and RPM)
bool canbusIsReceiving = false; // can bus shield is receiving from the can bus (all data)- triggered with interrupt

//PINS
int powerONJetsonRelay_PIN = 5; // when engine is ON, set to high. 
// DO NOT USE PIN 6 OR 11 
// PIN 6 IS CONNECTED TO THE NUILD-IN LED, THAT WE TURN ON WHEN SWITCHING ON WIFI
// When using 11 it stops sending UDP msgs.
int canbusINTERRUPT_PIN = 7; // check mkr canbus shield documentation

//timekeepeing
#define seconds() (millis() / 1000)

void setup() {
  Serial.begin(115200);//115200);  // initialize serial communication
  // while (!Serial);       //starts the program if we open the serial monitor.


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

  pinMode(canbusINTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(canbusINTERRUPT_PIN), CanBusInterruptFunction, CHANGE);
  pinMode(powerONJetsonRelay_PIN, OUTPUT);
  digitalWrite(powerONJetsonRelay_PIN, 0);

  //connectToNetwork(); // connect to wifi if available
  //time
  //Serial.println(seconds());
}

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connectToNetwork();
  }

  //udpReceive();

  // do the main can-bus sniffing
  // try to parse packet

  int packetSize = CAN.parsePacket();

  if (packetSize) {

    if (!canbusIsReceiving) {
      sendUDPData(String("cs:CanBus is Active"));
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
      if (canbus_rpm > 0) {
        digitalWrite(powerONJetsonRelay_PIN, 1); // switch on power for jetson
      }
      // RPM messages come many times within a second
      // when engine is turned off, they stop comming
      // with engine but power on, other messages may arrive
      // So we use the RPM messages as indication of car engine running
      lastTimeRPMReceivedInSeconds = seconds();
    }
  }

  //  // test
  delay(1000);
  if (canbus_speed < 500) {
    canbus_speed = canbus_speed + 10;
    canbus_rpm = canbus_rpm + 10;
    Serial.println("Sending data " + String(canbus_speed));
    sendUDPData();
    sendUDPData(String("cs:Engine is ON"));
    digitalWrite(powerONJetsonRelay_PIN, 1);
  } else {
    delay(15000);
    canbus_speed = 0;
    canbus_rpm = 0;
    sendUDPData();
    sendUDPData(String("cs:Engine is OFF"));
    digitalWrite(powerONJetsonRelay_PIN, 0);
    delay(10000);
  }

  if (newCanbusData) {
    sendUDPData();
  }

  if (WiFi.status() == WL_CONNECTED) {  // if a central is connected to the peripheral
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
      canbus_speed = 0;
      canbus_rpm = 0;
      sendUDPData(); // sent the last values. especially RPM 0
      sendUDPData(String("cs:Engine is OFF"));
      delay(10000);// delay, so we can shutdown processes in jetson, before turning off power
      digitalWrite(powerONJetsonRelay_PIN, 0);
    }
    canbusIsReceiving = false;
    newCanbusData = false;
  }
}

void udpReceive() {
  // if there's data available, read a packet

  int packetSize = Udp.parsePacket();

  if (packetSize) {

    // read the packet into packetBufffer

    int len = Udp.read(packetBuffer, 255);

    if (len > 0) {

      packetBuffer[len] = 0;

    }

    Serial.println("Contents:");

    Serial.println(packetBuffer);
  }
}

void connectToNetwork() {
  // try to connect to the network:


  for (byte networkCounter = 0; networkCounter < sizeof(SSIDs) / sizeof(SSIDs[0]); networkCounter++) {
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(SSIDs[networkCounter]);       // print the network name (SSID);
    // Connect to WPA/WPA2 network:
    if (WiFi.begin(SSIDs[networkCounter], WiFiPasswords[networkCounter]) == WL_CONNECTED) {
      break;  //connected to network
    }
  }
  if ( WiFi.status() == WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, HIGH);
    // print the SSID of the network you're attached to:
    if (Serial) Serial.print("Connected to: ");
    if (Serial) Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    if (Serial) Serial.print("IP Address: ");
    if (Serial) Serial.println(ip);
    // get broadcast ip
    broadcastAddress = getBroadcastIP();
    if (Serial) Serial.println(broadcastAddress);
    Udp.begin(port);
    sendUDPData(String("cs:Connected with ip " + String(ip)));
  }
}

IPAddress getBroadcastIP() {
  IPAddress broadcastIp = WiFi.localIP();
  broadcastIp[3] = 255;
  return broadcastIp;
}

void splitString(String str, String stringToSplitWith) {
  Serial.println(str);
  String strs[4];
  int StringCount = 0;
  // Split the string into substrings
  while (str.length() > 0)
  {
    int index = str.indexOf(stringToSplitWith);
    if (index == -1) // No space found
    {
      strs[StringCount++] = str;
      break;
    }
    else
    {
      strs[StringCount++] = str.substring(0, index);
      str = str.substring(index + 1);
    }
  }

  // Show the resulting substrings
  for (int i = 0; i < StringCount; i++)
  {
    Serial.print(i);
    Serial.print(": \"");
    Serial.print(strs[i]);
    Serial.println("\"");
  }
}

void CanBusInterruptFunction() {
  int val = 0;
  val = digitalRead(canbusINTERRUPT_PIN);
  sendUDPData(String("cs:CanBus pin value = " + String(val)));

}

void sendUDPData(String str) {
  if ( WiFi.status() == WL_CONNECTED) {
    // start a new packet:
    Udp.beginPacket(broadcastAddress, port);
    Udp.println(str);    // add payload to it
    Udp.endPacket();     // finish and send packet
  }
}

void sendUDPData(void) {
  if ( WiFi.status() == WL_CONNECTED) {
    String dataStr = String("speed:" + String(canbus_speed));
    sendUDPData(dataStr);

    dataStr = String("rpm:" + String(canbus_rpm));
    sendUDPData(dataStr);

    if (canbus_speed > 0) {
      // read accelaration data
      acceleration();
      dataStr = String("gx:" + String(gyro_x_accel));
      sendUDPData(dataStr);
    }
  }
}

void CanBusReceive(void) {

  // do the main can-bus sniffing
  // try to parse packet

  int packetSize = CAN.parsePacket();

  if (packetSize) {

    if (!canbusIsReceiving) {
      sendUDPData(String("cs:CanBus is Active"));
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
      if (canbus_rpm > 0) {
        digitalWrite(powerONJetsonRelay_PIN, 1); // switch on power for jetson
      }
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
