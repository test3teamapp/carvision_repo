
char buffer[10];

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop()
{

  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s10.0");
    Serial.println("a-3.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s20.0");
    Serial.println("a4.0"); 
    delay(100);
    sprintf(buffer, "s%d", 25);
    Serial.println(buffer); 
    sprintf(buffer, "a%d", 5);
    Serial.println(buffer); 
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s30.0");
    Serial.println("a0.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s50.0");
    Serial.println("a9.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s70.0");
    Serial.println("a0.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s100.0");
    Serial.println("a4.0");
    delay(100);
  }
}
