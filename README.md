# Arduino bi-directional DShot
> Bi-directional Dshot300 on an Arduino.

This is an implementation of DShot 300 on an Arduino. It can transmit (uninverted) DShot and inverted Dshot in order for the ESC to respond with a telemetry packet.

## Motivation
The motivation behind this project is to have an easy way to debug DShot communication with an ESC, especially bi-directional DShot. This is not meant as a library to use Dshot on your Arduino but is rather intended for testing purposes.

Apart from that I was interested to see how far I can push an Arduino when it comes to very timing sensitive tasks without resorting to assembler, and I must say, I was very surprised how well this worked out. Although some tricks (like directly reading ports instead of digitalRead) have been used.

> Yes - it is definetly possible to have a better optimized version in assembler. I want this to be easily understandable so I can come back to it in a couple of years and still understand what is happening.

## Limitations
The Arduino as such is very limited in regards to receiving bi-directional packages since they come in very short burst - on Dshot 300 the response frame arrives in less than 60us and can effectively only be processed via pin D8 - which is the "Input Capture" pin.

D8 is responsible for sending the DShot signal and receiving the response package.

### Manipulaltion
The Dshot value sent by default is **0**. Via serial console other values (up to 2047) might be provided.

> Be aware that due to the nature of Dshot frames being sent continuously it might happen that serial Input is not processed immediately or only chunks arrive. There is not much that can be done which would not involve messing with the timing of the DShot frames being sent/received.

## How does it work?
A timer triggers sending of the DShot frame via interrupt then the following things happen:

1. D8 is set as output
2. Dshot frame is sent
3. D8 is set as input
4. Input capture on timer 1 is used to capture the time between falling and rising endges
5. Conversion of the captured data to 21 bit response message
6. Decoding of response message
7. Serial print of decoded response message

## Usage
I recommend you use Visual Studio Code with the platform IO plugin. A **platformio.ini** is included in the repository with a working serial monitor configuration.

## C2 Interface
This sketch also includes a C2 interface for your convenience, allowing to read, write and erase an EFM8 MCU. This will require you to install the EFM8 python client. Also by default the Arduino will start in DShot mode, to enable the C2 interface, pull pin 13 low and reset the arduino.

To switch back to DShot mode, remove the jumper to low form pin 13 and reset the arduino.

### Installing python EFM8 client
use pipenv to create an virtual env, install dependencies and enter the env:
```
pipenv install
pipenv shell
```

then run the client like so:

```
python efm8.py read /dev/ttyUSB0 output.hex
python efm8.py write /dev/ttyUSB0 input.hex
python efm8.py erase
```

## Compatibility
This should work on any Arduino Uno or clone running at 16MHz. If you are using a different Arduino board, make sure you are using (and setting) the pin which has the "Input Capture" functionality.

## Resources
* [DShot - the missing handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)

## Contributions
Contributions are very welcome. Feel free to submit a PR against the develop branch. If you have any questions or feature requests, feel free to [open an issue](https://github.com/bird-sanctuary/arduino-bi-directional-dshot/issues).
