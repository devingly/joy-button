// will play many audio file formats, mp3,aac,flac etc.
// See github page : https://github.com/schreibfaul1/ESP32-audioI2S

#include "Arduino.h"
#include "Audio.h"
#include "SD.h"
#include "FS.h"

// SD Card
#define SD_CS          5
#define SPI_MOSI      23    
#define SPI_MISO      19
#define SPI_SCK       18

// I2S
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

Audio audio;
String mp3Path;

void setup() {
  Serial.begin(115200);
  Serial.println("setup...");
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!SD.begin(SD_CS))
  {
    Serial.println("Error talking to SD card!");
    while (true); // end program
  }
  else
  {
    Serial.println("SD connected");
  }

  // TODO: get mp3Path through configuration
  mp3Path = "/test.mp3";
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(15); // 0...21
  audio.connecttoFS(SD, mp3Path);
  Serial.println("connected to audio");
  Serial.print("track: ");
  Serial.println(mp3Path);
}

void loop()
{
  audio.loop();
}
