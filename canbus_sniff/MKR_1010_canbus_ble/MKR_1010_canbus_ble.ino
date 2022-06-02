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

BLEService carService("0000dd30-76d9-48e9-aa47-d0538d18f701");  // creating the service

BLEUnsignedIntCharacteristic speedReading("0000dd31-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the Speed Value characteristic
BLEUnsignedIntCharacteristic angleReading("0000dd32-76d9-48e9-aa47-d0538d18f701", BLERead | BLENotify);  // creating the Steering angle Value characteristic

long currentMillis = 0;
long previousMillis = 0;
int canbus_speed = 0;
int canbus_steeringangle = 0;
int canbus_dataload[8];
BLEDevice central;
bool bleClientConnected = false;
bool newCanbusData = false;


void setup() {
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

  BLE.setLocalName("MYSERVICE");  //Setting a name that will appear when scanning for bluetooth devices
  BLE.setAdvertisedService(carService);

  carService.addCharacteristic(speedReading);  //add characteristics to a service
  carService.addCharacteristic(angleReading);

  BLE.addService(carService);  // adding the service

  speedReading.writeValue(0);  //set initial value for characteristics
  angleReading.writeValue(0);

  BLE.advertise();  //start advertising the service
  Serial.println("Bluetooth device active, waiting for connections...");
  //previousMillis = millis();
  bleClientConnected = false;
  newCanbusData = false;
}

void loop() {
   // currentMillis = millis();
    if (!central){
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

    if (CAN.packetId() == 0x610) {
   
      int i=0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }
      if (canbus_dataload[1] != 12 && canbus_dataload[2] != 80) {
        canbus_speed =  canbus_dataload[2];      
        Serial.print("s");
        Serial.print(canbus_dataload[2]);
        Serial.println();
        newCanbusData = true;
      }
    }

    if (CAN.packetId() == 0x260) {
     
      int i=0;
      while (CAN.available() && i < packetSize) {
        canbus_dataload[i] = CAN.read();
        i++;
      }

      canbus_steeringangle =  canbus_dataload[5];      
      Serial.print("a");
      Serial.print(canbus_dataload[5]);
      Serial.println();
      newCanbusData = true;
      
    }   
  }

  if (central && central.connected()) {  // if a central is connected to the peripheral
    //Serial.print("Connected to central: ");

    //Serial.println(central.address());  // print the central's BT address

    digitalWrite(LED_BUILTIN, HIGH);  // turn on the LED to indicate the connection

    if (newCanbusData){
    speedReading.writeValue(canbus_speed);
    angleReading.writeValue(canbus_steeringangle);
        //Serial.println("sending ble data 1");
    }
    
 }else {

    digitalWrite(LED_BUILTIN, LOW);  // when the central disconnects, turn off the LED
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
  }
  
}
