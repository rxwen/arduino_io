# arduino_io 

[![Build Status](https://travis-ci.org/rxwen/arduino_io.svg?branch=master)](https://travis-ci.org/rxwen/arduino_io)
[travis-ci.org](https://travis-ci.org/)

use arduino as io device

checks input/adc status and drives output pin according to received command from uart.

It's highly recommended to use [platformio](http://platformio.org) instead of arduino ide. Install with ```pip install platformio``` command.

## build instructions
```
cd directory
platformio upgrade                      # Only once
platformio init --board megaADK         # Only once
platformio lib install ArduinoJson      # Only once, install dependencies

platformio run                          # optional, for compiling test
platformio run -t upload                # for burn to arduino
```
