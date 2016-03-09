# arduino_io 

[![Build Status](https://travis-ci.org/rxwen/arduino_io.svg?branch=master)](https://travis-ci.org/rxwen/arduino_io)

use arduino as io device

reports input events to uart port and drives output pin according to received command from uart.

It's highly recommended to use [platformio](http://platformio.org) instead of arduino ide. Install with ```pip install platformio``` command.

## build instructions
```
cd directory
platformio init --board uno
platformio run
platformio run -t upload
```
