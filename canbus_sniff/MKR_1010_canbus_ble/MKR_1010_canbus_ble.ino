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
#include <DFRobot_LIS2DH12.h>

BLEService carService("0000dd30-76d9-48e9-aa47-d0538d18f701");  // creating the service

BLEUnsignedIntCharacteristic speedReading("0000dd31-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the Speed Value characteristic
BLEUnsignedIntCharacteristic angleReading("0000dd32-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the Steering angle Value characteristic
BLEUnsignedIntCharacteristic accelReading("0000dd33-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the accelaration angle Value characteristic
BLEUnsignedIntCharacteristic rpmReading("0000dd34-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);    // creating the rpm Value characteristic

long currentMillis = 0;
long previousMillis = 0;
int canbus_speed = 0;
int canbus_steeringangle = 0;
int accelaration_angle = 0;
int canbus_rpm = 0;
int canbus_dataload[8];
BLEDevice central;
bool bleClientConnected = false;
bool newCanbusData = false;
bool skipSteeringAngleMeassurement = 0;  // we skip a few (countdown counter) since it botllenecks the ble

DFRobot_LIS2DH12 LIS;  //Accelerometer


void setup() {
  Wire.begin();        // i2c dfrobot accelerometer id =  0x018
  Serial.begin(9600);  // initialize serial communication
  //while (!Serial);       //starts the program if we open the serial monitor.


  //initialize BLE library
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

  // initiate accelerometer
  // Set measurement range
  // Ga: LIS2DH12_RANGE_2GA
  // Ga: LIS2DH12_RANGE_4GA
  // Ga: LIS2DH12_RANGE_8GA
  // Ga: LIS2DH12_RANGE_16GA
  while (LIS.init(LIS2DH12_RANGE_16GA) == -1) {  //Equipment connection exception or I2C address error
    Serial.println("No I2C devices found");
    delay(1000);
  }

  BLE.setLocalName("MYSERVICE");  //Setting a name that will appear when scanning for bluetooth devices
  BLE.setAdvertisedService(carService);

  carService.addCharacteristic(speedReading);  //add characteristics to a service
  carService.addCharacteristic(angleReading);
  carService.addCharacteristic(accelReading);
  carService.addCharacteristic(rpmReading);

  BLE.addService(carService);  // adding the service

  speedReading.writeValue(0);  //set initial value for characteristics
  angleReading.writeValue(0);
  accelReading.writeValue(0);
  rpmReading.writeValue(0);

  BLE.advertise();  //start advertising the service
  Serial.println("Bluetooth device active, waiting for connections...");
  //previousMillis = millis();
  bleClientConnected = false;
  newCanbusData = false;
}

void loop() {
  // currentMillis = millis();
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
    // if (CAN.packetRtr()) {
    // Remote transmission request, packet contains no data
    //   Serial.print("RTR ");
    // only print packet data for non-RTR packets
    // }

    //Serial.print("packet with id 0x");
    //Serial.print(CAN.packetId(), HEX);


    // Designation 	        ID 	    bit-start 	bit-count
    //speed (high refresh) 	0x0b4 	40 	        16
    //speed (low refresh) 	0x610 	8 	        16

    if (CAN.packetId() == 0x610) {  // SPEED  (low frequency) e.g.  id      data length   data
                                    //                              0x610     8           20 00 00 64 C0 00 00 00 497

      int i = 0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }
      if (canbus_dataload[1] != 12 && canbus_dataload[2] != 80) {  // when turning on/off the lights bytes 1 and 2 are switching momentarily
        canbus_speed = word(canbus_dataload[2], canbus_dataload[3]) / 100;
        Serial.print("Speed = ");
        Serial.print(canbus_speed);
        Serial.println();
        newCanbusData = true;
      }
    }

    if (CAN.packetId() == 0x260) {  // STEERING ANGLE e.g.  id      data length   data
                                    //                      0x260   8             00 00 00 00 00 FF 56 BF 19
      if (skipSteeringAngleMeassurement == 0) {
        skipSteeringAngleMeassurement = 5;
        int i = 0;
        while (CAN.available() && i < packetSize) {
          canbus_dataload[i] = CAN.read();
          i++;
        }

        if (canbus_dataload[5] > 0) {  // when 0, steering angle not moved.
                                       // byte 5 is taking values 255,254,253... etc when wheel moved clockwise
                                       // byte 5 is taking values 1, 2, 3... etc when wheel moved anticlockwise
          canbus_steeringangle = canbus_dataload[5];
          Serial.print("Steering angle = ");
          Serial.print(canbus_dataload[5]);
          Serial.println();
          newCanbusData = true;
        }
      } else {
        skipSteeringAngleMeassurement = skipSteeringAngleMeassurement - 1;
      }
    }
    // 	                          ID 	    bit-start 	bit-count
    // engine-rev (low refresh) 	0x3b3 	0 	        16
    if (CAN.packetId() == 0x3B3) {  // RPM  e.g.  id      data length   data
                                    //            0x3B3    3            05 4E 28

      int i = 0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }


      canbus_rpm = word(canbus_dataload[0], canbus_dataload[1]);
      Serial.print("RPM = ");
      Serial.print(canbus_rpm);
      Serial.println();
      newCanbusData = true;
    }
  }

  // read accelaration data

  acceleration();

  if (central && central.connected()) {  // if a central is connected to the peripheral
    //Serial.print("Connected to central: ");

    //Serial.println(central.address());  // print the central's BT address

    digitalWrite(LED_BUILTIN, HIGH);  // turn on the LED to indicate the connection

    if (newCanbusData) {
      speedReading.writeValue(canbus_speed);
      angleReading.writeValue(canbus_steeringangle);
      accelReading.writeValue(accelaration_angle);
      rpmReading.writeValue(canbus_rpm);
      //Serial.println("sending ble data 1");
    }

  } else {

    digitalWrite(LED_BUILTIN, LOW);  // when the central disconnects, turn off the LED
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
  }
}


/*!
 *  @brief Print the position result.
 */
void acceleration(void) {
  int16_t x, y, z;

  delay(100);
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);

  if (x > 50 || y > 50) {
    accelaration_angle = round(atan2(y, x) * 180 / 3.14159265);
    Serial.print("angle of velocity : ");  //print acceleration angle
    Serial.println(accelaration_angle);
  }
  /*
  Serial.print("Acceleration x: "); //print acceleration
  Serial.print(x);
  Serial.print(" mg \ty: ");
  Serial.print(y);
  Serial.print(" mg \tz: ");
  Serial.print(z);
  Serial.println(" mg");
  */
}