#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(9600);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setListenOnlyMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

// For speed the message is like this:
// ID   DLC    DATA
// 610  8      20 00 16 E4 C0 00 00 00 

// from https://github.com/P1kachu/talking-with-cars/blob/master/notes/toyota-yaris.md

/*
Speed
Designation 	        ID 	    bit-start 	bit-count
speed (high refresh) 	0x0b4 	40 	        16
speed (low refresh) 	0x610 	8 	        16
*/

// The part that represents the speed value are the bits 9 to 24 
// in our example message "00 16"

/* the message is stored in the following stracture :
struct can_frame {
    uint32_t can_id;  // 32 bit CAN_ID + EFF/RTR/ERR flags 
    uint8_t  can_dlc;
    uint8_t  data[8];
};
*/

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    /*  
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();   
    */

    // we are looking for 0x610 id
    if (canMsg.can_id == 0x610) {
      if (canMsg.data[1] != 12 && canMsg.data[2] != 80) { // this is for avoiding 
      //a spike in the values when the car engine start/stops 
      // and the lights turn on / off

        //Serial.print(canMsg.data[1]);
        //Serial.print("Speed = ");
        //Serial.print(canMsg.data[2]);
        //Serial.println();
        // to jetson
        // For all intent and purposes, data[2] containing the first 0-255 values
        // for km/hour is enough for getting the speed of a 1000cc yaris
         //Serial.print("s");
        //Serial.println(canMsg.data[2]);   

        char buffer[10];
        sprintf(buffer, "s%d", canMsg.data[2]);
        Serial.println(buffer);         
      }
    }

    if (canMsg.can_id == 0x3B3) {

      // Serial.print("RPM = " );
      // Serial.print(canMsg.data[0]);
      // Serial.print(" ");
      // Serial.print(canMsg.data[1]);
      // Serial.print(" ");
      // Serial.print(canMsg.data[2]);
      // Serial.println();
    }

    if (canMsg.can_id == 0x260) {
      // Serial.print("Steering wheel angle = " );

      // for (int i = 0; i < canMsg.can_dlc; i++)  { // print the data
      //   Serial.print(canMsg.data[i]);
      //   Serial.print(" ");
      // }
      // Serial.println();
    }
  }
}
