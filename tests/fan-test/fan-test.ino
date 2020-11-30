// will play many audio file formats, mp3,aac,flac etc.
// See github page : https://github.com/schreibfaul1/ESP32-audioI2S

#include "Arduino.h"

#define FAN_PIN 14

void setup() {
  Serial.begin(115200);
  Serial.println("setup...");
  pinMode(FAN_PIN, OUTPUT);
}

void loop()
{
  Serial.println("fan on for 5...");
  digitalWrite(FAN_PIN, HIGH);
  delay(5000);

  Serial.println("fan off for 5...");
  digitalWrite(FAN_PIN, LOW);
  delay(5000);
}
