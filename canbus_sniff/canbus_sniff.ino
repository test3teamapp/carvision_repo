
const char* candata[] = {
 "DFFFFFFF 3 FE FF FF",
"DFFFFFFF 8 FF FF FF FF FF FF FE FF",
"DFFFEFFF 8 FB FF FF FF FF FF BF FF",
"DFFFEFFF 8 FB FF FF FF FF FF 8F FE",
"DFFAFFFF 8 FF FF FF FF FF FF BF FF",
"DFFFFFFF 5 FF FF FB FF FF",
"DFFFFFFF 3 FF FF FF",
"DFFFEFFF 8 FB FF FF FF FF FF BF FE",
"DFFFFFFF 8 FF FF FF FF FF FF FE FF",
"DFFFFFFF 8 FB FF FF FF FF FF BF FB",
"DFDDFFEF 7 FF BF FF FF EB FE FF",
"DFFFFFFF 8 FB FF FF FF F7 FF F8 FF",
"DFFFFFFF 8 FF FF BF FF FF AF FA FF",
"DFFFEFFF 8 FB FF FF FF FF FF BB FF",
"DFFFFFFF 8 FF FF FF FB FF FF FA FF",
"DFFFFFFF 8 EF FF FF EB FF FE BF FF",
"DFFFFFFF 3 FF FF FF",
"DFFFEBFF 8 FB FF FF BF FB FF 8F FA",
"DFFFFFFF 6 FF FF BF FF F7 A7",
"DFDDFFFF 8 FF FF FF FF FF FE FF FF",
"DFBFFFFF 3 FC FF FF",
"400007FF 8 FF FF FF FF BF FB FF F8",
"DFFFFFFF 3 FF FF FF",
"400007FF 8 FF EB FF FF FA FF FE BF",
"DFFFEFFF 8 FB FF FF FF FF FF BF FB",
"DFFEFFFF 3 FF FF FF",
"DFBEBFFF 3 F7 FF FF",
"DFFFFFFF 8 FF FF FF FF FF AF FE FF",
"9FFFFFFF 8 FF FF FF BF FB FF FE FF",
"DFFFEFFF 8 FF FF FF FF FF FF AF FF",
"DFFFEFFF 8 FB FF FF EF FF FF FF FF"};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  
}

void loop() {
  // put your main code here, to run repeatedly:

  for (int cnt = 0; cnt < 31; cnt++) {
    delay(10);
    Serial.println(candata[cnt]);
  }
  delay(200);
}
