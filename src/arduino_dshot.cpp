#include "Arduino.h"
#include "Dshot.h"

FREQUENCY frequency = F500;

// Set inverted to true if you want bi-directional DShot
bool inverted = true;

// DSHOT Output pin
unsigned int pinDshot = 8;

/* Initialization */
unsigned long dshotResponse = 0;
unsigned long dshotResponseLast = 0;
uint16_t mappedLast = 0;

// Buffer for counting duration between falling and rising edges
const uint8_t buffSize = 20;
uint16_t counter[buffSize];

// Statistics for success rate
uint16_t receivedPackets = 0;
uint16_t successPackets = 0;

Dshot dshot = new Dshot(inverted);
volatile uint16_t frame = dshot.buildFrame(0, 0);

#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)

void sendBitsDshot300();
void sendBitDshot300(uint8_t bit);
void sendBitDshot300Inverted(uint8_t bit);

void sendBitsDshot300() {
  // Send Dshot frame
  uint16_t temp = frame;
  uint8_t offset = 0;
  do{
    if(inverted) {
      sendBitDshot300Inverted((temp & 0x8000) >> 15);
    } else {
      sendBitDshot300((temp & 0x8000) >> 15);
    }
    temp <<= 1;
  } while(++offset < 0x10);

  // Set to Input in order to process the response - this will be at 3.3V level
  pinMode(pinDshot, INPUT_PULLUP);

  // Delay around 26us
  DELAY_CYCLES(410);

  register uint8_t ices1High = 0b01000000;
  register uint16_t prevVal = 0;
  register uint8_t tifr;
  register uint16_t *pCapDat;

  TCCR1A = 0b00000001; // Toggle OC1A on compare match
  TCCR1B = 0b00000010; // trigger on falling edge, prescaler 8, filter off 
  TIFR1 = (1 << ICF1) | (1 << OCF1A) | (1 << TOV1); // clear all timer flags
  
  // Limit to 90us - that should be enough to fetch the whole response
  // at 2MHz - scale factor 11 - 180 ticks should be enough.
  OCR1A = 180;
  TCNT1 = 0x00;

  for(pCapDat = counter; pCapDat <= &counter[buffSize - 1];) {
    // wait for edge or overflow (output compare match)
    while(!(tifr = (TIFR1 & ((1 << ICF1) | (1 << OCF1A))))) {}

    uint16_t val = ICR1;

    // Break if counter overflows
    if(tifr & (1 << OCF1A)) {
      // Ignore overflow at the beginning of capture
      if(pCapDat != counter) {
        break;
      }
    }

    TCCR1B ^= ices1High; // toggle the trigger edge
    TIFR1 = (1 << ICF1) | (1 << OCF1A); // clear input capture and output compare flag bit
    
    *pCapDat = val - prevVal;

    prevVal = val;
    pCapDat++;
  }

  pinMode(pinDshot, OUTPUT);

  // Set all 21 possible bits to one and flip the once that should be zero
  dshotResponse = 0x001FFFFF;
  unsigned long bitValue = 0x00;
  uint8_t bitCount = 0;
  for(uint8_t i = 1; i < buffSize; i += 1) {
    bitValue ^= 0x01; // Toggle bit value - always start with 0
    counter[i] = (counter[i] + 1) / 5; // TODO: Less than optimal, ranges would probably be better here...

    for(uint8_t j = 0; j < counter[i]; j += 1) {
      dshotResponse ^= (bitValue << (20 - bitCount++));
    }
  }

  // Decode GCR 21 -> 20 bit
  dshotResponse ^= dshotResponse >> 1;
}

/**
 * digitalWrite takes about 3.4us to execute, that's why we switch ports directly.
 * Switching ports directly will allow a transition in 0.19us or 190ns.
 * 
 * In an optimal case, without any lag for sending a "1" we would switch high, stay high for 2500 ns (40 ticks) and then switch back to low.
 * Since a transition takes some time too, we need to adjust the waiting period accordingly. Ther resulting values have been set using an
 * oscilloscope to validate the delay cycles.
 * 
 * Duration for a single byte should be 1/300kHz = 3333.33ns = 3.3us or 53.3 ticks
 * 
 * The delays after switching back to low are to account for the overhead of going through the loop ins sendBitsDshot*
 */
void sendBitDshot300Inverted(uint8_t bit) {
  if(bit) {
    PORTB = B00000000;
    //DELAY_CYCLES(40);
    DELAY_CYCLES(37);
    PORTB = B00000001;
    //DELAY_CYCLES(13);
    DELAY_CYCLES(7);
  } else {
    PORTB = B00000000;
    //DELAY_CYCLES(20);
    DELAY_CYCLES(16);
    PORTB = B00000001;
    //DELAY_CYCLES(33);
    DELAY_CYCLES(25);
  }
}

void sendBitDshot300(uint8_t bit) {
  if(bit) {
    PORTB = B00000001;
    //DELAY_CYCLES(40);
    DELAY_CYCLES(37);
    PORTB = B00000000;
    //DELAY_CYCLES(13);
    DELAY_CYCLES(7);
  } else {
    PORTB = B00000001;
    //DELAY_CYCLES(20);
    DELAY_CYCLES(16);
    PORTB = B00000000;
    //DELAY_CYCLES(33);
    DELAY_CYCLES(25);
  }
}

void setupTimer() {
  cli();

  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;

  if(frequency == F500) {
    // 500 Hz (16000000/((124 + 1) * 256))
    OCR2A = 124;
    TCCR2B |= 0b00000110; // Prescaler 256
  } else if(frequency == F1k) {
    // 1000 Hz (16000000/((124 + 1) * 128))
    OCR2A = 124;
    TCCR2B |= 0b00000101; // Prescaler 128
  } else if(frequency == F2k) {
    // 2000 Hz (16000000/((124 + 1) * 64))
    OCR2A = 124;
    TCCR2B |= 0b00000100; // Prescaler 64
  } else if(frequency == F4k) {
    // 4000 Hz (16000000/( (124 + 1) * 32))
    OCR2A = 124;
    TCCR2B |= 0b00000011; // Prescaler 32
  } else {
    // 8000 Hz (16000000/( (249 + 1) * 8))
    OCR2A = 249;
    TCCR2B |= 0b00000010; // Prescaler 8
  }

  TCCR2A |= 0b00001010; // CTC mode - count to OCR2A
  TIMSK2 = 0b00000010;  // Enable INT on compare match A

  sei();
}

ISR(TIMER2_COMPA_vect) {
  sendBitsDshot300();
}

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(pinDshot, OUTPUT);

  // Set pin high or low depending on inversion
  PORTB = B00000000;
  if(inverted) {
    PORTB = B00000001;
  }

  Serial.println("Input throttle value to be sent to ESC");
  Serial.println("Valid throttle values are 47 - 2047");
  Serial.print("Frames are sent repeatadly in the chosen update frequency: ");
  Serial.print(frequency);
  Serial.println("Hz");

  setupTimer();
}

void loop() {
  // Serial read might not always trigger properly here since the timer might interrupt
  // Disabling the interrupts is not an option since Serial uses interrupts too.
  if(Serial.available() > 0) {
    uint16_t dshotValue = Serial.parseInt(SKIP_NONE);
    Serial.read();

    if (dshotValue > 2047) {
      dshotValue = 2047;
    }
    frame = dshot.buildFrame(dshotValue, 0);

    Serial.print("> Frame: ");
    Serial.print(frame, BIN);
    Serial.print(" Value: ");
    Serial.println(dshotValue);
  }

  // Re-calculate the response values if DShot response did in fact change
  if(dshotResponse != dshotResponseLast) {
    uint16_t mapped = dshot.mapTo16Bit(dshotResponse);

    uint8_t crc = mapped & 0x0F;
    uint16_t value = mapped >> 4;
    uint8_t crcExpected = dshot.calculateCrc(value);

    uint32_t periodBase = value & 0b0000000111111111;
    uint8_t periodShift = value >> 9 & 0b00000111;
    uint32_t periodTime =  periodBase << periodShift;

    // Calculation of success rate is a bit flawed since it will not update for every response
    // If we want to havae this, we would need to move calculation to the interrupt too.
    receivedPackets++;
    if(crc == crcExpected) {
      successPackets++;
      Serial.print("OK: ");
    } else {
      Serial.print("--: ");
    }

    // Reset packet count if overflows
    if(!receivedPackets) {
      successPackets = 0;
    }
    float successPercent = (successPackets * 1.0 / receivedPackets * 1.0) * 100;

    Serial.print(periodTime);
    Serial.print("us ");
    Serial.print(successPercent);
    Serial.println("%");
    
    dshotResponseLast = dshotResponse;
  }
}
