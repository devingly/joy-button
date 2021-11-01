//From bildr article: http://bildr.org/2012/08/rotary-encoder-arduino/
// NOTE: encoder-test2 is much better! use that one
#include "Arduino.h"

#define LEFT -1
#define RIGHT 1

// D2 = INT0; D3 = INT1;  --> need interrupts for encoder
#define ENCODER_PIN_A GPIO_NUM_34
#define ENCODER_PIN_B GPIO_NUM_35
#define ENCODER_SWITCH_PIN GPIO_NUM_32

struct Encoder
{
  volatile bool pressed;
  volatile bool rotated;
  volatile int pressedCount;
  volatile int direction; // 1 = RIGHT/CLOCKWISE; -1 = LEFT/COUNTER_CLOCKWISE, 0 = NONE
  volatile int lastEncodedValue;  // 4 bit value -- e.g. `0100`
  volatile long lastReadingMillis; // for debouncing
};

Encoder encoder = {false, false, 0, 0, 0b0000, 0};

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
    encoder.direction = 0;  // default to 0 (NONE)
    if (encoder.lastEncodedValue == 0 && currentEncodedValue == 1) encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 1 && currentEncodedValue == 3) encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 3 && currentEncodedValue == 2) encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 2 && currentEncodedValue == 0) encoder.direction = RIGHT;
    if (encoder.lastEncodedValue == 0 && currentEncodedValue == 2) encoder.direction = LEFT;
    if (encoder.lastEncodedValue == 2 && currentEncodedValue == 3) encoder.direction = LEFT;
    if (encoder.lastEncodedValue == 3 && currentEncodedValue == 1) encoder.direction = LEFT;
    if (encoder.lastEncodedValue == 1 && currentEncodedValue == 0) encoder.direction = LEFT;

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
  Serial.begin(115200);                 // start the serial monitor link
  pinMode(ENCODER_PIN_A, INPUT_PULLUP); // set ENCODER_PIN_A as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(ENCODER_PIN_B, INPUT_PULLUP); // set ENCODER_PIN_B as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(ENCODER_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SWITCH_PIN), encoderButtonPressed, FALLING); // want to know when button is coming down from HIGH to LOW when it is released after a press
}

void loop()
{
  if (encoder.pressed)
  {
    encoder.pressed = false;
    Serial.printf("encoder has been pressed %u times\n", encoder.pressedCount);
  }

  if (encoder.rotated)
  {
    encoder.rotated = false;
    if (encoder.direction == RIGHT) {
      Serial.println("rotated RIGHT");
    }
    else if (encoder.direction == LEFT) {
      Serial.println("rotated LEFT");
    }
  }
}