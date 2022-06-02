    //
    // text size 2 gives 15-1 rows and 26-1 colums 
    //
    //
// Include files    
    #include <SPI.h>          // f.k. for Arduino-1.5.2
    #include "Adafruit_GFX.h"// Hardware-specific library
    #include <MCUFRIEND_kbv.h>
    #include <can.h>
    #include <mcp2515.h>
    #include <CanHacker.h>
    #include <CanHackerLineReader.h>
    #include <lib.h>
    #include <SoftwareSerial.h>

// Assign human-readable names to some common 16-bit color values:
    #define BLACK 0x0000
    #define NAVY 0x000F
    #define DARKGREEN 0x03E0
    #define DARKCYAN 0x03EF
    #define MAROON 0x7800
    #define PURPLE 0x780F
    #define OLIVE 0x7BE0
    #define LIGHTGREY 0xC618
    #define DARKGREY 0x7BEF
    #define BLUE 0x001F
    #define GREEN 0x07E0
    #define CYAN 0x07FF
    #define RED 0xF800
    #define MAGENTA 0xF81F
    #define YELLOW 0xFFE0
    #define WHITE 0xFFFF
    #define ORANGE 0xFD20
    #define GREENYELLOW 0xAFE5
    #define PINK 0xF81F
    #define LCD_CS A3 // Chip Select goes to Analog 3
    #define LCD_CD A2 // Command/Data goes to Analog 2
    #define LCD_WR A1 // LCD Write goes to Analog 1
    #define LCD_RD A0 // LCD Read goes to Analog 0
    #define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
    #ifndef min
    #define min(a, b) (((a) < (b)) ? (a) : (b))
    #endif
    #define BME_SCK 13
    #define BME_MISO 12
    #define BME_MOSI 11
    #define BME_CS 10
    #define SEALEVELPRESSURE_HPA (1013.25)

// Setup Modules
    MCUFRIEND_kbv tft;
    MCP2515 mcp2515(53);
    //SoftwareSerial softwareSerial(SS_RX_PIN, SS_TX_PIN);

// Setup Global Varibles
    int canid;
    int line = 0;
    int data[8];
    String TIME = "";
    String DATE = "";

    char daysOfTheWeek[7][12] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

    const char hex_asc_upper[] = "0123456789ABCDEF";

    unsigned long delayTime;
    uint16_t randomNumber = 0;
    const int SPI_CS_PIN = 53;
    const int INT_PIN = 2;
    const int SS_RX_PIN = 3;
    const int SS_TX_PIN = 4;

    struct can_frame canMsg;

    unsigned long pgnID = 0;
    CanHackerLineReader *lineReader = NULL;
    CanHacker *canHacker = NULL;


void setup() {
    Serial.begin(115200);
    while (!Serial);
    SPI.begin();
    //softwareSerial.begin(115200);

// default settings
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS);
  mcp2515.setNormalMode();

    
// Get & Print TFT Info to Serial monitor
    Serial.println();
        uint16_t ID = tft.readID(); //
    Serial.print("ID = 0x");
    Serial.println(ID, HEX);
    if (ID == 0xD3D3) ID = 0x9481; // write-only shield
//  ID = 0x9329;                             // force ID
    Serial.print("width = ");
    Serial.println(tft.width());
    Serial.print("Height = ");
    Serial.println(tft.height());
    
// print CAN data headder
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");

// Start TFT Screen   
    tft.begin(ID);
    tft.setRotation(3);
    tft.setTextSize(2);
    tft.fillScreen(BLACK);
    tft.setTextColor(WHITE, BLACK);
    tft.invertDisplay(false);
    tft.setCursor(0, 0);
    titleScreen(tft.width(), tft.height(), ID);
    tft.fillScreen(BLACK);
}

void loop() {
    //------------------------------------------------------------------------------
if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) 
    {
    
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" ");
    unsigned long pngID = canMsg.can_id;
// Reduce ID To 4 digits (HEX xxxx)
    pngID = pngID << 8;
    pngID = pngID >> 16;
    pgnID = pngID;
    Serial.print(pngID, HEX);
    Serial.print(" ");
    int dataLen = canMsg.can_dlc;
    // print DLC
    Serial.print(dataLen, HEX); 
    Serial.print(" ");
    for (int i = 0; i<canMsg.can_dlc; i++) 
        {  
            // print the data
            data[i] = canMsg.data[i];
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

/*     pgnID = 0xf004;

    tft.setCursor(0, 1*16);
for (int i = 0; i<8; i++) 
        {  
            // print the data
            data[i] = random(0xff);
            Serial.print(data[i], HEX);
            Serial.print(" ");
        }
        Serial.println(" "); */
    //------------------------------------------------------------------------------
//Line 0(top line of lcd)
    //Date & Time (From ECU)
    if (pgnID == 0xfee6){
        String Day = "  ";
        unsigned int Date = data[4]/4;
        unsigned int Month = data[3];
        unsigned int Year = data[5]+1985;
        unsigned int Hour = data[2];
        unsigned int Minute = data[1];
        unsigned int Second = data[0] /4;

    //Add leading 0's to date month and time and build up a time/date string
        String DATE = "";
        DATE = " ";
          if (Date < 10)
                    DATE = DATE + (" ");
          DATE = DATE + String(Date) + "/";
          if (Month < 10)
                    DATE = DATE + "0";
        DATE = DATE + String(Month) + "/" + String(Year);

    String TIME = "        ";
          if (Hour < 10)
                    TIME = TIME + (" ");
          TIME = TIME + String(Hour) + ":";
          if (Minute < 10)
                    TIME = TIME + ("0");
          TIME = TIME + String(Minute) + "  ";
          
    tft.setCursor(0, 0);
    tft.setTextColor(WHITE, RED);
    tft.print(DATE);
    tft.print(TIME);
    tft.setTextColor(WHITE, BLACK);

    //Serial.println(DATE);
    //Serial.println(TIME);
    }

//line1
    else if (pgnID == 0xf005){
   // printData();
    tft.setTextColor(WHITE, BLACK);
    
    
    tft.setCursor(0, 1*16);
    unsigned int gear = data[0]-125;
    if (gear < 0) gear = gear * -1;
    int dir = data[7];
    String Gear = "Gear " + String(gear) + (char)dir + " ";
    print_bar(1, 10, gear, 12, RED);
    tft.println(Gear );
    }
    
// Line2
    else if (pgnID == 0xf004){
        unsigned int d1 = data[2] - 125;
        unsigned int power = d1;
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 2*16);
        print_bar(2, 10, power, 250, GREEN);
        tft.println("Power    ");
    //    }

// Line3
//    else if (pgnID == 0xffff){
        unsigned int r1 = data[3];
        unsigned int r2 = data[4];
        float rpm = ( r1/ 8) + ((r2*256) / 8);
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 3 * 16);
        //Speed
        int colour = DARKCYAN;
        if (rpm > 900) colour = GREEN;
        if (rpm > 1600) colour = BLUE;
        if (rpm > 2100) colour = RED;

        print_bar(3, 10, rpm, 2700,colour );
        tft.println("RPM      ");
    }
//Line4
    else if (pgnID == 0xfef1){
      //  printData();
      unsigned int s1 = data[1];
      unsigned int s2 = data[2];
      float speed = ((s1) + (s2 * 256)) / 256;
      tft.setTextColor(WHITE, BLACK);
      tft.setCursor(0, 4 * 16);
      int colour = GREEN;
      if (speed > 90)
          colour = RED;
      print_bar(4, 10, speed, 130, colour);
      tft.println("Speed    ");
    }
// Line5
    else if (pgnID == 0xfef2){
        unsigned int s0 = data[0];
        unsigned int s3 = data[1];
        float flow = ((s0) + (s3 * 256))*0.05;
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 5 * 16);
        print_bar(5, 10, flow, 100, GREEN);
        tft.setCursor(0, 5 * 16);
        tft.println("Fuel "+String(flow));
   // }
// Line6
    //else if (pgnID == 0xffff){
        unsigned int s1 = data[2];
        unsigned int s2 = data[3];
        float mpg = (0xfaff)*((s1) + (s2 * 256));
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 6 * 16);
        print_bar(6, 10, mpg, 100, CYAN);
        tft.println("inst " + String(mpg));
    }
// Line7
    else if (pgnID == 0xf000){
        unsigned int r1 = data[1];
        int retarder = (125 - r1);
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 7 * 16);
        print_bar(7, 10, retarder, 115, MAGENTA);
        tft.println("Retarder  ");
    }
// Line8
    else if (pgnID == 0xfeef){
      //  printData();
        unsigned int oilp = data[3] ;
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 8 * 16);
        print_bar(8, 10, oilp, 128, YELLOW);
        tft.println("oil Pres");
   }
// Line9
   else if (pgnID == 0xfec1){
       long  km= 0;
        unsigned int k4 = data[0];
        unsigned int k3 = data[1];
        unsigned int k2 = data[2];
        unsigned int k1 = data[3];
        km = k4 + (k3 * 0x100);
        km = km + (k2 * 0x010000);
        km = km + (k1 * 0x01000000);
        

        km = km/2;
        km = km;
        printData();
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 9 * 16);
        
        tft.println("Km        "+String((float)km/100));
        
    }
// Line10
    else if (pgnID == 0xffff){
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(0, 10*16);
    print_bar(10, 10, randomNumber, 100, MAROON);
    tft.println("1");
    }
// Line11
    else if (pgnID == 0xfe70){
        printData();
        unsigned int w1 = data[3];
        unsigned int w2 = data[2];
        unsigned int gvv = (w1 * 256);
        gvv = gvv + w2;
        String GVW = String(gvv)+"0";
        if (data[3]>0xfa)
            GVW = "No DATA";
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 10 * 16);
        tft.println("GVW       " + GVW+"      ");
        //print_bar(11, 10, randomNumber, 100, GREEN);
    }
// Line12
    else if (pgnID == 0xFEAE){
    //printData();
        unsigned int air = data[2];
        tft.setTextColor(WHITE, BLACK);
        tft.setCursor(0, 11 * 16);
        air = air * 0.078125;
        int colour = GREEN;
        if (air <8) colour = YELLOW;
        if (air <=5) colour = RED;
        print_bar(11, 10, int(air), 14, colour);
        tft.println("Air    "+String(int(air))+" ");
    }

// Line13
else if (pgnID == 0xfefc) {

    unsigned int fuel = data[1];
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(0, 12*16);
    unsigned int used = 0xfa - fuel;
    used = used * 7;
    int colour = GREEN;
    if (fuel < 0x25){
        colour = RED;
    }
    print_bar(12, 10, int(fuel), 260, colour);
    tft.println("Fuel  " + String(int(fuel*1.6))+" ");
}

else if (pgnID==0xfe56){
    
    unsigned int adblu = data[0];
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(0, 13*16);
    adblu = adblu * 0.25;
    int colour = BLUE;
    if (adblu < 25){
        colour = RED;
    }
    print_bar(13, 10, int(adblu), 65, colour);
    tft.println("adblue "+String(int(adblu))+" ");
}
else {}
}


void print_bar(unsigned int row, unsigned int col, unsigned int val, unsigned int maxval, int Colour) {
    // Rows =14 Colums=25
    row = (row * 16)+2;
    col = col * 12;
    float m = 0;
    //Serial.print(String(m)+"  |  ");
    m = 315 - col;
    //Serial.print(String(m)+"  |  ");
    m = m / maxval;
    //Serial.print(String(m)+"  |  ");
    m = m * val;
    //Serial.println(String(m));
    //Serial.println("Speed " + String(val) + " max " + String(maxval) + " Percentage bar " + String(m) + " " );
    //Serial.println(" ");
    tft.writeFillRect(col, row, (int) m, 13,  Colour);
    tft.writeFillRect(col+m, row, 193 -(int) m, 13, BLACK);
    }
void printData(){
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(0, (14*16)+0);

    tft.print(pgnID, HEX);
    tft.print("   ");
    tft.print(data[0], HEX);
    tft.print(" ");
    tft.print(data[1], HEX);
    tft.print(" ");
    tft.print(data[2], HEX);
    tft.print(" ");
    tft.print(data[3], HEX);
    tft.print(" ");
    tft.print(data[4], HEX);
    tft.print(" ");
    tft.print(data[5], HEX);
    tft.print(" ");
    tft.print(data[6], HEX);
    tft.print(" ");
    tft.print(data[7], HEX);
    tft.println(" ");
}
void titleScreen(unsigned int width, unsigned int Height, unsigned int ID){
    
    tft.print("ID     ");
    tft.setTextColor(GREEN, BLACK);
    tft.print("= 0x");
    tft.println(ID, HEX);
    tft.setTextColor(WHITE, BLACK);
    tft.print("width  = ");
    tft.setTextColor(GREEN, BLACK);
    tft.println(tft.width());
    tft.setTextColor(WHITE, BLACK);
    tft.print("Height = ");
    tft.setTextColor(GREEN, BLACK);
    tft.println(tft.height());
    tft.setTextColor(WHITE, BLACK);
    tft.setTextSize(3);
    tft.setTextColor(MAGENTA, BLACK);
    tft.println(" ");
    tft.println("   CAN Monitor");
    tft.println(" ");
    tft.println("       By ");
    tft.println("Gary Metheringham");
    
    tft.setTextSize(2);
    tft.setCursor(0, 15 * 16);
    tft.setTextColor(WHITE, BLACK);
    while(!mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK){
        tft.setCursor(8*13, 13 * 16);
        tft.print("No Data");
        tft.setTextColor(RED, BLACK);
        delay(100);
        tft.setCursor(8*13, 13 * 16);
        tft.print("No Data");
        tft.setTextColor(WHITE, BLACK);
        delay(100);
    }    
}