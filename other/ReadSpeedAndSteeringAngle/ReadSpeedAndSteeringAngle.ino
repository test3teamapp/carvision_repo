
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
    Serial.println("s10");
    delay(50);
    Serial.println("a9");
    delay(50);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s20");
    delay(50);
    Serial.println("a4");
    delay(50);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s30");
    delay(50);
    Serial.println("a2");
    delay(50);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s50");
    delay(50);
    Serial.println("a0");
    delay(50);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s70");
    delay(50);
    Serial.println("a0");
    delay(50);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s100");
    delay(50);
    Serial.println("a0");
    delay(50);
  }
}
