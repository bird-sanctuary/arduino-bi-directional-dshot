#include "C2.h"

C2::C2(volatile uint8_t *port, uint8_t pinCk, uint8_t pinD, uint8_t pinLed) {
  _port = port;
  _pinCk = pinCk;
  _pinD = pinD;
  _pinLed = pinLed;
}

void C2::init() {
  reset();

  writeAddress(0x02);
  writeData(0x02);
  writeData(0x04);
  writeData(0x01);
}

void C2::reset() {
  *_port &= ~(1 << _pinCk);
  delayMicroseconds(50);
  *_port |= (1 << _pinCk);
  delayMicroseconds(50);
}

uint8_t C2::readBits(uint8_t length) {
  uint8_t mask = 0x01 << (length - 1);
  uint8_t data = 0;

  DDRD &= ~(1 << _pinD);
  PIND &= (1 << _pinD);
  for (uint8_t i = 0; i < length; i += 1) {
    clockPulse();

    data >>= 1;
    if (PIND & (1 << _pinD)) {
      data = data | mask;
    }
  }
  DDRD |= (1 << _pinD);

  return data;
}

void C2::sendStopBit() {
  sendBits(0x00, 1);
}

void C2::sendAddressReadInstruction() {
  uint8_t data = 0x00 | (0x00 << 0) | (ADDRESS_READ << 1);
  sendBits(data, 3);
}

void C2::sendDataReadInstruction(uint8_t byte) {
  uint8_t data = 0x00 | (0x00 << 0) | (DATA_READ << 1) | ((byte - 1) << 3);
  sendBits(data, 5);
}

void C2::sendAddressWriteInstruction() {
  uint8_t data = 0x00 | (0x00 << 0) | (ADDRESS_WRITE << 1);
  sendBits(data, 3);
}

void C2::sendDataWriteInstruction(uint8_t byte) {
  uint8_t data = 0x00 | (0x00 << 0) | (DATA_WRITE << 1) | ((byte - 1) << 3);
  sendBits(data, 5);
}

void C2::sendBits(uint8_t data, uint8_t length) {
  for(uint8_t i = 0; i < length; i += 1) {
    if(data & 0x01) {
      *_port |= (1 << _pinD);
    } else {
      *_port &= ~(1 << _pinD);
    }

    clockPulse();
    data >>= 1;
  }
}

void C2::clockPulse() {
  *_port &= ~(1 << _pinCk);
  *_port |= (1 << _pinCk);
}

uint8_t C2::readByte() {
  return readBits(8);
}

void C2::sendByte(uint8_t byte) {
  sendBits(byte, 8);
}

uint8_t C2::readAddress() {
  sendAddressReadInstruction();
  uint8_t retval = readByte();
  sendStopBit();

  return retval;
}

void C2::writeAddress(uint8_t address) {
  sendAddressWriteInstruction();
  sendByte(address);
  sendStopBit();
}

void C2::writeData(uint8_t data) {
  sendDataWriteInstruction(1);
  sendByte(data);
  while (readBits(1) == 0) {}
  sendStopBit();
}

/**
 * Response for the C2 interface is indicated by a one bit after an arbitary number
 * of 0 bits. The next 8 bit after the first one are the response byte.
 */
uint8_t C2::waitForResponse() {
  while(readBits(1) == 0) {}
  return readBits(8);
}

uint8_t C2::readData() {
  sendDataReadInstruction(1);
  uint8_t response = waitForResponse();
  sendStopBit();

  return response;
}

uint8_t C2::pollBitHigh(uint8_t mask) {
  uint8_t retval;

  do {
    retval = readAddress();
  } while ((retval & mask) == 0);

  return retval;
}

uint8_t C2::pollBitLow(uint8_t mask) {
  uint8_t retval;

  do {
    retval = readAddress();
  } while (retval & mask);

  return retval;
}

uint8_t C2::readFlashBlock(uint16_t address, uint8_t *data, uint8_t bytes) {
  writeAddress(FPDAT);
  writeData(BLOCK_READ);

  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  uint8_t value = readData();
  if(value != 0x0D) {
    return EXIT_FAILURE;
  }

  // Write high byte of address
  writeData(address >> 8);
  pollBitLow(_inBusy);

  // Write low byte of address
  writeData(address & 0xFF);
  pollBitLow(_inBusy);

  writeData(bytes);
  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  readData();

  for(uint8_t i = 0; i < bytes; i += 1) {
    pollBitHigh(_outReady);
    data[i] = readData();
  }

  return EXIT_SUCCESS;
}

uint8_t C2::writeFlashBlock(uint16_t address, uint8_t *data, uint8_t length) {
  writeAddress(FPDAT);
  writeData(BLOCK_WRITE);

  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  uint8_t value = readData();
  if(value != 0x0D) {
    return EXIT_FAILURE;
  }

  // Write high byte of address
  writeData(address >> 8);
  pollBitLow(_inBusy);

  // Write low byte of address
  writeData(address & 0xFF);
  pollBitLow(_inBusy);

  writeData(length);
  pollBitLow(_inBusy);

  for(uint8_t i = 0; i < length; i += 1) {
    writeData(data[i]);
    pollBitLow(_inBusy);
  }
  pollBitHigh(_outReady);

  value = readData();
  if(value != 0x0D) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

uint8_t C2::eraseDevice() {
  writeAddress(FPDAT);
  writeData(DEVICE_ERASE);

  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  uint8_t value = readData();
  if(value != 0x0D) {
    return EXIT_FAILURE;
  }

  writeData(0xDE);
  pollBitLow(_inBusy);

  writeData(0xAD);
  pollBitLow(_inBusy);

  writeData(0xA5);
  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  value = readData();
  if(value != 0x0D) {
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

uint8_t C2::getState() {
  return _state;
}

void C2::resetState() {
  _state = 0;
}

uint8_t C2::updateState(uint8_t data) {
  switch(_state) {
    case 0x00: {
      _messagePtr = 0;
      _message[_messagePtr++] = data;

      _state = 1;
    } break;

    case 0x01: {
      _bytesLeft = data;
      _message[_messagePtr++] = data;

      if(_bytesLeft == 0) {
        _state = 3;

        break;
      }

      _state = 2;
    } break;

    case 0x02: {
      _message[_messagePtr++] = data;
      _bytesLeft--;

      if(_bytesLeft == 0) {
        _state = 3;
      }
    } break;
  }

  return _state;
}

volatile uint8_t *C2::getMessage() {
  return _message;
}

void C2::setup() {
  Serial.begin(1000000);

  DDRD = 0x00;
  DDRD |= (1 << _pinD) | (1 << _pinCk);

  *_port = 0x00;
  *_port |= (1 << _pinCk);

  digitalWrite(_pinLed, LOW);
}

void C2::loop() {
  unsigned char crc;
  unsigned char newcrc;
  unsigned long address;

  if(Serial.available()) {
    uint8_t data = Serial.read();
    updateState(data);

    if(_state == 3) {
      switch(_message[0]) {
        case 0x00: {
          Serial.write(0x80);
        } break;

        case 0x01: {
          init();

          Serial.write(0x81);
          digitalWrite(_pinLed, HIGH);

          resetState();
        } break;

        case 0x02: {
          reset();

          Serial.write(0x82);
          digitalWrite(_pinLed, LOW);

          resetState();
        } break;

        case 0x03: {
          address = (((unsigned long)(_message[4]))<<8) + (((unsigned long)(_message[5]))<<0);
          crc = _message[6];
          newcrc = _message[5] + _message[4];
          for(uint8_t i = 0; i < _message[2]; i+= 1) {
            _flashBuffer[i] = _message[i+7];
          }


          for(uint8_t i = 0; i < _message[2]; i += 1) {
            newcrc += _flashBuffer[i];
          }

          if(crc != newcrc) {
            Serial.write(0x43);
            break;
          }

          uint8_t ch = _message[2];
          writeFlashBlock(address, _flashBuffer, ch);
          Serial.write(0x83);

          resetState();
        } break;

        case 0x04: {
          eraseDevice();
          Serial.write(0x84);

          resetState();
        } break;

        case 0x05: {
          Serial.write(0x85);

          address = (((unsigned long)(_message[3]))<<16) + (((unsigned long)(_message[4]))<<8) + (((unsigned long)(_message[5]))<<0);
          readFlashBlock(address, _flashBuffer, _message[2]);
          for(uint8_t i = 0; i < _message[2]; i += 1) {
            Serial.write(_flashBuffer[i]);
          }

          resetState();
        } break;

        case 0x06: {
          writeAddress(_message[3]);
          writeData(_message[4]);
          Serial.write(0x86);

          resetState();
        } break;

        case 0x07: {
          writeAddress(_message[3]);
          Serial.write(readData());
          Serial.write(0x87);

          resetState();
        } break;
      }
    }
  }
}