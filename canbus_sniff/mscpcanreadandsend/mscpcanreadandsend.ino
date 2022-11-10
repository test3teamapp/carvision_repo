#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
        Serial.print(canMsg.can_id, HEX); // print ID
        Serial.print(" ");
        Serial.print(canMsg.can_dlc, HEX); // print DLC
        Serial.print(" ");
    
       for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
          Serial.print(canMsg.data[i],HEX);
          Serial.print(" ");
       }

    if (canMsg.can_id == 0x610) {
      if (canMsg.data[1] != 12 && canMsg.data[2] != 80) {
        //Serial.print(canMsg.data[1]);
        Serial.print("Speed = ");
        Serial.print(canMsg.data[2]);
        Serial.println();
      }
    }

    if (canMsg.can_id == 0x3B3) {
      //Serial.print(canMsg.data[1]);
      Serial.print("RPM = " );
      Serial.print(canMsg.data[0]);
      Serial.print(" ");
      Serial.print(canMsg.data[1]);
      Serial.print(" ");
      Serial.print(canMsg.data[2]);
      Serial.println();
    }

    if (canMsg.can_id == 0x260) {
      Serial.print("Steering wheel angle = " );

      for (int i = 0; i < canMsg.can_dlc; i++)  { // print the data
        Serial.print(canMsg.data[i]);
        Serial.print(" ");
      }
      Serial.println();
    }

  }

  // -----send -----

  struct can_frame frame;
  frame.can_id = 0x000;
  frame.can_dlc = 4;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;

  /* send out the message to the bus and
    tell other devices this is a standard frame from 0x00. */
  if (mcp2515.sendMessage(&frame) == MCP2515::ERROR_OK){
    Serial.println("Std Msg sent");
  }

  delay(1000);

  struct can_frame frame2;
  frame2.can_id = 0x12345678 | CAN_EFF_FLAG;
  frame2.can_dlc = 2;
  frame2.data[0] = 0xFF;
  frame2.data[1] = 0xFF;

  /* send out the message to the bus using second TX buffer and
    tell other devices this is a extended frame from 0x12345678. */
  if (mcp2515.sendMessage(MCP2515::TXB1, &frame2) == MCP2515::ERROR_OK){
    Serial.println("Ext Msg sent");
  }

  delay(1000);

}
