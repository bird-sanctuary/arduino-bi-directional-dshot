#include "Arduino.h"
#include "C2.h"

/**
 * which was based on: https://gist.github.com/racerxdl/c9a592808acdd9cd178e6e97c83f8baf
 * which was based on: https://github.com/jaromir-sukuba/efm8prog/
 */


// GPIO is manipulated through PORT mappings for better speed.
// Flashes EFM8 at about 10kB/s
// Baud rate: 1000000

#define PORT PORTD
#define C2D_PIN 2
#define C2CK_PIN 3

#define LED LED_BUILTIN

C2 *c2 = new C2(&PORTD, (uint8_t) C2CK_PIN, (uint8_t) C2D_PIN, (uint8_t) LED_BUILTIN);

void setup() {
  c2->setup();
}

void loop() {
  c2->loop();
}
