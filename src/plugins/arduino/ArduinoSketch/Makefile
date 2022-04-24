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

