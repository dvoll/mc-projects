/*
 * Example using the Rotary library, dumping integers to the serial
 * port. The integers increment or decrement depending on the direction
 * of rotation.
 *
 * This example uses polling rather than interrupts.
 */
#include <Arduino.h>
#include <Rotary.h>

#define PIN_KEY 21

// Rotary encoder is wired with the common to ground and the two
// outputs to pins 5 and 6.
Rotary rotary = Rotary(18, 19);


// Counter that will be incremented or decremented by rotation.
int counter = 0;

bool pressed = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_KEY, INPUT);
}

void loop() {
  if (digitalRead(PIN_KEY) == LOW) {
    if (pressed == 0) {
      pressed = 1;
      Serial.print ("Key pressed; ");
      counter = 0;
      Serial.println(counter);
    }
  } else {
    pressed = 0;
  }
  unsigned char result = rotary.process();
  if (result == DIR_CW) {
    counter++;
    Serial.println(counter);
  } else if (result == DIR_CCW) {
    counter--;
    Serial.println(counter);
  }
}

