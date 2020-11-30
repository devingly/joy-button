#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "Audio.h"
#include "SD.h"
#include "FS.h"

// LCD
#define LCD_SDA 21
#define LCD_SCL 22
#define FAN_PIN 14
// ENCODER
#define ENCODER_PIN_A GPIO_NUM_34
#define ENCODER_PIN_B GPIO_NUM_35
#define ENCODER_SWITCH_PIN GPIO_NUM_32
#define LEFT -1
#define RIGHT 1
// SD
#define SD_CS 5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18
// I2S/MP3
#define I2S_DOUT 25
#define I2S_BCLK 27
#define I2S_LRC 26

#define MARKER_LENGTH 9   // number of markers in the song at which to change states

struct Encoder
{
  volatile bool pressed;
  volatile bool rotated;
  volatile int pressedCount;
  volatile int direction;          // 1 = RIGHT/CLOCKWISE; -1 = LEFT/COUNTER_CLOCKWISE, 0 = NONE
  volatile int lastEncodedValue;   // 4 bit value -- e.g. `0100`
  volatile long lastReadingMillis; // for debouncing
};

struct Marker
{
  long startAt;   // elapsed millis() after which marker instruction should be followed
  bool markerComplete;
  bool machineOff;
  bool audioPlay;
  bool audioStop;
  bool fanOn;
  bool fanOff;
  bool displayOn;
  bool displayOff;
  String displayLine1;
  String displayLine2;
};

struct Machine
{
  bool isOn;
  long elapsed;
  long startedAt;
  struct Marker markers[MARKER_LENGTH];
};

struct Music
{
  bool shouldPlay;
  String fileToPlay;
};

Audio audio;
String mp3Path;
LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display
Encoder encoder = {false, false, 0, 0, 0b0000, 0};
Machine machine;
Music music = {false, "/test.mp3"};

void IRAM_ATTR updateEncoder()
{
  /*
    Pin A and Pin B will report HIGH/LOW in the following sequence of pairs as we rotate RIGHT:
    binary  [00, 01, 11, 10]
    decimal [ 0,  1,  3,  2]

    LAST  CURRENT   DIRECTION
    ==========================
     0  ->   1        RIGHT
     1  ->   3        RIGHT
     3  ->   2        RIGHT
     2  ->   0        RIGHT
     0  ->   2        LEFT
     2  ->   3        LEFT
     3  ->   1        LEFT
     1  ->   0        LEFT
  */

  // simple debounce
  const long currentMillis = millis();
  if (currentMillis - encoder.lastReadingMillis > 150)
  {
    int MSB = digitalRead(ENCODER_PIN_A); //MSB = most significant bit
    int LSB = digitalRead(ENCODER_PIN_B); //LSB = least significant bit
    //converting the 2 pin value to single number
    int currentEncodedValue = (MSB << 1) | LSB;
    encoder.direction = 0; // default to 0 (NONE)
    if (encoder.lastEncodedValue == 0 && currentEncodedValue == 1)
      encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 1 && currentEncodedValue == 3)
      encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 3 && currentEncodedValue == 2)
      encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 2 && currentEncodedValue == 0)
      encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 0 && currentEncodedValue == 2)
      encoder.direction = LEFT;
    if (encoder.lastEncodedValue == 2 && currentEncodedValue == 3)
      encoder.direction = LEFT;
    if (encoder.lastEncodedValue == 3 && currentEncodedValue == 1)
      encoder.direction = LEFT;
    if (encoder.lastEncodedValue == 1 && currentEncodedValue == 0)
      encoder.direction = LEFT;

    if (encoder.direction != 0)
    {
      Serial.printf("encoder.lastEncodedValue: %d   currentEncodedValue: %d    direction: %d\n", encoder.lastEncodedValue, currentEncodedValue, encoder.direction);

      encoder.lastEncodedValue = currentEncodedValue; //store this value for comparison next time
      encoder.lastReadingMillis = currentMillis;
      encoder.rotated = true;
    }
  }
}

void IRAM_ATTR encoderButtonPressed()
{
  // simple debounce
  const long currentMillis = millis();
  if (currentMillis - encoder.lastReadingMillis > 150)
  {
    encoder.pressedCount++;
    encoder.lastReadingMillis = currentMillis;
    encoder.pressed = true;
  }
}

void setup()
{
  Serial.begin(115200);
  // LCD
  Wire.begin(LCD_SDA, LCD_SCL);
  lcd.init();

  // ENCODER
  // pinMode(ENCODER_PIN_A, INPUT_PULLUP); // set ENCODER_PIN_A as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  // pinMode(ENCODER_PIN_B, INPUT_PULLUP); // set ENCODER_PIN_B as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(ENCODER_SWITCH_PIN, INPUT_PULLUP);
  // TODO: add hardware filter to each pin and retry this in examples version of encoder
  // attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SWITCH_PIN), encoderButtonPressed, FALLING); // want to know when button is coming down from HIGH to LOW when it is released after a press

  // FAN
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW); // off by default

  // MP3
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  if (!SD.begin(SD_CS))
  {
    Serial.println("Error talking to SD card!");
    while (true)
      ; // end program
  }
  else
  {
    Serial.println("SD connected");
  }

  // TODO: get mp3Path through configuration
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(15); // 0...21
  audio.connecttoFS(SD, music.fileToPlay);
  Serial.println("connected to audio");
  Serial.print("track: ");
  Serial.println(music.fileToPlay);

  // setup markers
  //                    startAt  markerComplete  machineOff  audioPlay  audioStop  fanOn  fanOff  displayOn  displayOff  displayLine1        displayLine2
  machine.markers[0] = {0,       false,          false,      true,      false,     false, false,  true,      false,      "       .        ", "                "};
  machine.markers[1] = {1000,    false,          false,      false,     false,     false, false,  true,      false,      "       ..       ", "                "};
  machine.markers[2] = {2000,    false,          false,      false,     false,     false, false,  true,      false,      "       ...      ", "                "};
  machine.markers[3] = {3000,    false,          false,      false,     false,     false, false,  true,      false,      "       ....     ", "                "};
  machine.markers[4] = {4000,    false,          false,      false,     false,     false, false,  true,      false,      "     Joy        ", "   is coming... "};
  machine.markers[5] = {5000,    false,          false,      false,     false,     true,  false,  false,     false,      "                ", "                "};
  machine.markers[6] = {10000,   false,          false,      false,     false,     false, true,   true,      false,      "   Scanning     ", "quantum field..."};
  machine.markers[7] = {15000,   false,          false,      false,     false,     true,  false,  true,      false,      " Returning      ", "    joy...      "};
  machine.markers[8] = {30000,   false,          true,       false,     true,      false, true,   true,      true,       "                ", "                "};
  machine.isOn = false;
  machine.elapsed = 0;
  machine.startedAt = 0;
  Serial.println("setup complete");
  resetMachine();
}

void resetMachine() 
{
  audio.stopSong();
  machine.isOn = false;
  int i;
  for (i = 0; i < MARKER_LENGTH; i++)
  {
    machine.markers[i].markerComplete = false;
  }

  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("   Joy Button   ");
  lcd.setCursor(0, 1);
  lcd.print("      :-)       ");
}

void loop()
{
  if (encoder.pressed)
  {
    Serial.println("encoder.pressed");
    encoder.pressed = false;
    if (!machine.isOn)
    {
      // start machine
      // Serial.println("starting machine");
      // for (int i=0; i<16; i++)
      // {
      //   lcd.scrollDisplayLeft();
      //   delay(50);
      // }
      // lcd.clear();
      machine.isOn = true;
      machine.elapsed = 0;
      machine.startedAt = millis();
    }
    else
    {
      resetMachine();
    }
  }

  if (machine.isOn)
  {
    // set current elapsed
    machine.elapsed = millis() - machine.startedAt;
    int i;
    for (i = 0; i < MARKER_LENGTH; i++ )
    {
      // skip completed markers
      if (machine.markers[i].markerComplete) continue;
      // perform next incomplete marker instructions if we've reached it's startAt
      if (machine.elapsed >= machine.markers[i].startAt)
      {
        Serial.printf("processing marker %d, elapsed %d, startAt %d\n", i, machine.elapsed, machine.markers[i].startAt);

        if (machine.markers[i].machineOff)
        {
          Serial.printf("stopping machine, marker %d, elapsed %d\n", i, machine.elapsed);
          resetMachine();
          return;
        }
        
        if (machine.markers[i].audioPlay)
        {
          Serial.printf("music.shouldPlay, marker %d, elapsed %d\n", i, machine.elapsed);
          music.shouldPlay = true;
        } 
        if (machine.markers[i].audioStop) 
        {
          Serial.printf("stopping audio, marker %d, elapsed %d\n", i, machine.elapsed);
          music.shouldPlay = false; 
          audio.stopSong();
        }
        if (machine.markers[i].fanOn)
        {
          Serial.printf("starting fan, marker %d, elapsed %d\n", i, machine.elapsed);
          digitalWrite(FAN_PIN, HIGH);
        }
        if (machine.markers[i].fanOff) 
        {
          Serial.printf("stopping fan, marker %d, elapsed %d\n", i, machine.elapsed);
          digitalWrite(FAN_PIN, LOW);
        }
        if (machine.markers[i].displayOn) 
        {
          Serial.printf("display on, marker %d, elapsed %d\n", i, machine.elapsed);
          lcd.display();
          lcd.setCursor(0, 0);
          lcd.print(machine.markers[i].displayLine1);
          lcd.setCursor(0, 1);
          lcd.print(machine.markers[i].displayLine2);
        }
        if (machine.markers[i].displayOff)
        {
          Serial.printf("display off, marker %d, elapsed %d\n", i, machine.elapsed);
          lcd.noDisplay();
        }

        machine.markers[i].markerComplete = true;
      }
      // otherwise change no state
      break;
    }

    if (music.shouldPlay) audio.loop();
  }
}
