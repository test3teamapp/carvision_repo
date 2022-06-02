#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);


void setup() {
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setListenOnlyMode();

  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    //    Serial.print(canMsg.can_id, HEX); // print ID
    //    Serial.print(" ");
    //    Serial.print(canMsg.can_dlc, HEX); // print DLC
    //    Serial.print(" ");
    //
    //    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
    //      Serial.print(canMsg.data[i],HEX);
    //      Serial.print(" ");
    //    }

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
}
