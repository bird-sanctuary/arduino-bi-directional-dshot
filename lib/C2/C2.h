#include "Arduino.h"

#ifndef C2_H
#define C2_H

enum C2Instruction {
  DATA_READ     = 0x00,
  DATA_WRITE    = 0x01,
  ADDRESS_READ  = 0x02,
  ADDRESS_WRITE = 0x03,
};

enum C2Addresses {
  DEVICEID = 0x00,
  REVID    = 0x01,
  FPCTL    = 0x02,
  FPDAT    = 0xB4, // May be different for non EFM8 targets
};

enum C2Commands {
  DEVICE_ERASE = 0x03,
  BLOCK_READ   = 0x06,
  BLOCK_WRITE  = 0x07,
};

enum C2Devices {
  EFM8BB1  = 0x30,
  EFM8BB2  = 0x32,
  EFM8BB51 = 0x39,
};

enum Actions {
  ACK   = 0x00,
  INIT  = 0x01,
  RESET = 0x02,
  WRITE = 0x03,
  ERASE = 0x04,
  READ  = 0x05,
  INFO  = 0x08
};

struct Device {
  uint8_t id;
  uint8_t revision;
};

class C2 {
  public:
    C2(volatile uint8_t *port, volatile uint8_t *ddr, volatile uint8_t *pin, uint8_t pinCk, uint8_t pinD, uint8_t pinLed);

    void init();
    void reset();

    void deviceInfo();

    uint8_t readAddress();
    void writeAddress(uint8_t address);

    uint8_t readData();
    void writeData(uint8_t data);

    uint8_t readBits(uint8_t length);
    void sendBits(uint8_t data, uint8_t length);
    void clockPulse();

    uint8_t pollBitHigh(uint8_t mask);
    uint8_t pollBitLow(uint8_t mask);

    uint8_t readFlashBlock(uint16_t address, uint8_t *data, uint8_t bytes);
    uint8_t writeFlashBlock(uint16_t address, uint8_t *data, uint8_t length);
    uint8_t eraseDevice();

    void resetState();
    uint8_t getState();
    uint8_t updateState(uint8_t data);

    volatile uint8_t *getMessage();

    void sendByte(uint8_t byte);
    uint8_t readByte();

    void sendStopBit();
    uint8_t waitForResponse();

    void sendAddressReadInstruction();
    void sendDataReadInstruction(uint8_t byte);

    void sendAddressWriteInstruction();
    void sendDataWriteInstruction(uint8_t byte);

    void setup();
    void loop();

  private:
    volatile uint8_t *_port;
    volatile uint8_t *_ddr;
    volatile uint8_t *_pin;

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

    Device device;
};

#endif