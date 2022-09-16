#include "Arduino.h"

#ifndef C2_H
#define C2_H

class C2 {
  public:
    C2(volatile uint8_t *port, uint8_t pinCk, uint8_t pinD, uint8_t pinLed);

    void init();
    void reset();

    uint8_t readAddress();
    void writeAddress(uint8_t address);

    uint8_t readData();
    void writeData(uint8_t data);

    uint8_t readBits(uint8_t length);
    void sendBits(uint8_t data, uint8_t length);
    void clockPulse();

    uint8_t pollBitHigh(uint8_t mask);
    uint8_t pollBitLow(uint8_t mask);

    uint8_t readFlashBlock(uint16_t address, uint8_t *data, uint8_t length);
    uint8_t writeFlashBlock(uint16_t address, uint8_t *data, uint8_t length);
    uint8_t eraseDevice();

    void resetState();
    uint8_t getState();
    uint8_t updateState(uint8_t data);

    volatile uint8_t *getMessage();

    void setup();
    void loop();

  private:
    volatile uint8_t *_port;

    uint8_t _pinCk;
    uint8_t _pinD;
    uint8_t _pinLed;

    uint8_t _state = 0;

    const uint8_t _inBusy = 0x02;
    const uint8_t _outReady = 0x01;

    volatile uint8_t _message[300];
    volatile uint8_t _messagePtr;
    volatile uint8_t _bytesLeft;

    uint8_t _flashBuffer[300];
};

#endif