# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# Read the full text in the LICENSE.GPL file in the doc directory.

.PHONY: build flash clean accstepper cleanall comm verify safe_flash

# important immediate assignment (only if other makefiles are somehow included)
mkfile_path := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

DEV_ARDUINO ?= /dev/arduino

build: Compiled/ArduinoSketch.ino.hex

COMPILE_COMMAND = arduino-cli compile -b arduino:avr:uno --build-path=$(mkfile_path)/Compiled

Compiled/ArduinoSketch.ino.hex: ArduinoSketch.ino
	mkdir -p Compiled
	$(COMPILE_COMMAND) || $(COMPILE_COMMAND) # sometimes compilation breaks, due to https://github.com/arduino/arduino-builder/issues/67. Repeating compilation normally resolves the compilation error

flash: Compiled/ArduinoSketch.ino.hex
	arduino-cli upload --port $(DEV_ARDUINO) --fqbn arduino:avr:uno --input-dir Compiled --verify

clean:
	rm -rf Compiled

cleanall: clean
	rm -rf libraries

comm:
	minicom -D $(DEV_ARDUINO)

verify: Compiled/ArduinoSketch.ino.hex
	avrdude  -patmega328p -carduino -P$(DEV_ARDUINO) -B 10 -F -Uflash:v:$<:i

safe_flash: Compiled/ArduinoSketch.ino.hex # flash only if verify goes wrong
	avrdude  -patmega328p -carduino -P$(DEV_ARDUINO) -B 10 -F -Uflash:v:$<:i || avrdude  -patmega328p -carduino -P$(DEV_ARDUINO) -B 10 -F -Uflash:w:$<:i
