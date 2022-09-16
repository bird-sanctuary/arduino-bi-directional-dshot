#include "C2.h"

C2::C2(volatile uint8_t *port, uint8_t pinCk, uint8_t pinD, uint8_t pinLed) {
  //C2::C2(volatile uint8_t *port, uint8_t pinCk, uint8_t pinD) {
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
    data = data >> 1;
    if (PIND & (1 << _pinD)) {
      data = data | mask;
    }
  }
  DDRD |= (1 << _pinD);
  //pinMode(C2D, OUTPUT);

  return data;
}

void C2::sendBits(uint8_t data, uint8_t length) {
  DDRD |= (1 << _pinD);

  for(uint8_t i = 0; i < length; i += 1) {
    if(data & 0x01) {
      *_port |= (1 << _pinD);
    } else {
      *_port &= ~(1 << _pinD);
    }

    clockPulse();
    data = data >> 1;
  }
}

void C2::clockPulse() {
  *_port &= ~(1 << _pinCk);
  *_port |= (1 << _pinCk);
}

uint8_t C2::readAddress() {
  sendBits(0x00, 1);
  sendBits(0x02, 2);
  uint8_t retval = readBits(8);
  sendBits(0x00, 1);

  return retval;
}

void C2::writeAddress(uint8_t address) {
  sendBits(0x00, 1);
  sendBits(0x03, 2);
  sendBits(address, 8);
  sendBits(0x00, 1);
}

void C2::writeData(uint8_t data) {
  sendBits(0x00, 1);
  sendBits(0x01, 2);
  sendBits(0x00, 2);
  sendBits(data, 8);

  uint8_t retval = 0;
  while (retval == 0) {
    retval = readBits(1);
  }

  sendBits(0x00, 1);
}

uint8_t C2::readData() {
  sendBits(0x00, 1);
  sendBits(0x00, 2);
  sendBits(0x00, 2);

  uint8_t retval = 0;
  while (retval == 0) {
    retval = readBits(1);
  }

  retval = readBits(8);
  sendBits(0x00, 1);

  return retval;
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

uint8_t C2::readFlashBlock(uint16_t address, uint8_t *data, uint8_t length) {
  writeAddress(0xB4);
  writeData(0x06);
  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  uint8_t retval = readData();
  writeData(address >> 8);
  pollBitLow(_inBusy);
  writeData(address & 0xFF);

  pollBitLow(_inBusy);
  writeData(length);

  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  retval = readData();
  uint8_t i = 0;
  for(i = 0; i < length; i += 1) {
    pollBitHigh(_outReady);
    retval = readData();
    data[i] = retval;
  }

  return i;
}

uint8_t C2::writeFlashBlock(uint16_t address, uint8_t *data, uint8_t length) {
  writeAddress(0xB4);

  writeData(0x07);
  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  readData();

  writeData(address >> 8);
  pollBitLow(_inBusy);

  writeData(address & 0xFF);
  pollBitLow(_inBusy);

  writeData(length);
  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  uint8_t retval = readData();
  for(uint8_t i = 0; i < length; i += 1) {
    writeData(data[i]);
    pollBitLow(_inBusy);
  }
  pollBitHigh(_outReady);

  return retval;
}

uint8_t C2::eraseDevice() {
  writeAddress(0xB4);

  writeData(0x03);
  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  readData();

  writeData(0xDE);
  pollBitLow(_inBusy);

  writeData(0xAD);
  pollBitLow(_inBusy);

  writeData(0xA5);
  pollBitLow(_inBusy);
  pollBitHigh(_outReady);

  return readData();
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

        return 1;
    } break;

    case 0x01: {
        _bytesLeft = data;
        _message[_messagePtr++] = data;

        if(_bytesLeft == 0) {
          return 3;
        }

        return 2;
    } break;

    case 0x02: {
        _message[_messagePtr++] = data;
        _bytesLeft--;

        if(_bytesLeft == 0) {
          return 3;
        }
    } break;
  }

  return _state;
}

uint8_t *C2::getMessage() {
  return _message;
}

void C2::setup() {
  Serial.begin(1000000);

  DDRD = 0x00;
  *_port = 0x00;

  DDRD |= (1 << _pinD) | (1 << _pinCk);
  *_port |= (1 << _pinCk);

  digitalWrite(_pinLed, LOW);
}

void C2::loop() {
  unsigned char crc;
  unsigned char newcrc;
  unsigned long address;

  if(Serial.available()) {
    uint8_t data = Serial.read();
    _state = updateState(data);
    uint8_t *message = getMessage();

    if(_state == 3) {
      switch(message[0]) {
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
          address = (((unsigned long)(message[4]))<<8) + (((unsigned long)(message[5]))<<0);
          crc = message[6];
          newcrc = message[5] + message[4];
          for(uint8_t i = 0; i < message[2]; i+= 1) {
            _flashBuffer[i] = message[i+7];
          }


          for(uint8_t i = 0; i < message[2]; i += 1) {
            newcrc += _flashBuffer[i];
          }

          if(crc != newcrc) {
            Serial.write(0x43);
            break;
          }

          uint8_t ch = message[2];
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

          address = (((unsigned long)(message[3]))<<16) + (((unsigned long)(message[4]))<<8) + (((unsigned long)(message[5]))<<0);
          readFlashBlock(address, _flashBuffer, message[2]);
          for(uint8_t i = 0; i < message[2]; i += 1) {
            Serial.write(_flashBuffer[i]);
          }

          resetState();
        } break;

        case 0x06: {
          writeAddress(message[3]);
          writeData(message[4]);
          Serial.write(0x86);

          resetState();
        } break;

        case 0x07: {
          writeAddress(message[3]);
          Serial.write(readData());
          Serial.write(0x87);

          resetState();
        } break;
      }
    }
  }
}