#ifndef Dshot_H
#define Dshot_H

enum FREQUENCY {
  F500 = 500,
  F1k = 1000,
  F2k = 2000,
  F4k = 4000,
  F8k = 8000
};

class Dshot {
  public:
    Dshot(bool inverted);

    uint16_t buildFrame(uint16_t value, byte telemetry);
    uint8_t calculateCrc(uint16_t value);
    uint8_t revertMapping(uint16_t value);
    uint16_t mapTo16Bit(uint32_t value);

  private:
    bool _inverted;

    // Bit Masks for a Dshot frame
    const byte _crcMask = 0x0F;
    const byte _telemetryMask = 0x10;
    const uint16_t _valueMask = 0xFFE0;
};

#endif
