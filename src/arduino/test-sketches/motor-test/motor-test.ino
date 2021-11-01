#include "Arduino.h"
#define FAN_PIN 33

void setup()
{
  Serial.begin(115200);
  pinMode(FAN_PIN, OUTPUT);
}

void loop()
{
  Serial.println("ON");
  digitalWrite(FAN_PIN, HIGH);
  delay(5000);

  Serial.println("OFF");
  digitalWrite(FAN_PIN, LOW);
  delay(4000);
}