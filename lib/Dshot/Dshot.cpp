#include "Arduino.h"
#include "Dshot.h"

Dshot::Dshot(bool inverted) {
  _inverted = inverted;
}

uint8_t Dshot::calculateCrc(uint16_t value) {
  if(_inverted) {
    return ~(value ^ (value >> 4) ^ (value >> 8)) & _crcMask;
  }

  return (value ^ (value >> 4) ^ (value >> 8)) & _crcMask;
}

uint16_t Dshot::buildFrame(uint16_t value, byte telemetry) {
  uint16_t frame = 0x00;
  frame |= (value << 5) & _valueMask;
  frame |= (telemetry << 4) & _telemetryMask;
  frame |= calculateCrc(frame >> 4);

  return frame;
}

uint8_t Dshot::revertMapping(uint16_t value) {
  switch(value) {
    case 0x19: return 0x00;
    case 0x1B: return 0x01;
    case 0x12: return 0x02;
    case 0x13: return 0x03;
    case 0x1D: return 0x04;
    case 0x15: return 0x05;
    case 0x16: return 0x06;
    case 0x17: return 0x07;
    case 0x1A: return 0x08;
    case 0x09: return 0x09;
    case 0x0A: return 0x0A;
    case 0x0B: return 0x0B;
    case 0x1E: return 0x0C;
    case 0x0D: return 0x0D;
    case 0x0E: return 0x0E;
    case 0x0F: return 0x0F;
  }

  return 0xFF;
}

uint16_t Dshot::mapTo16Bit(uint32_t value) {
  uint16_t newValue;
  uint16_t mapped = 0x00;
  uint8_t leftShift = 0;

  for(int i = 0; i < 20; i += 5) {
    newValue = revertMapping(((value >> i) & 0x1F));
    mapped |= newValue << leftShift;
    leftShift += 4;
  }

  return mapped;
}