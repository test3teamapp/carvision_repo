
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
    Serial.println("s=10.0&a=1.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s=15.0&a=2.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s=20.0&a=3.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s=30.0&a=4.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, HIGH);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s=40.0&a=5.0");
    delay(100);
  }
  //digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i <= 25; i++)
  {
    Serial.println("s=55.0&a=6.0");
    delay(100);
  }
}
