.PHONY: build flash clean accstepper cleanall comm verify safe_flash

# important immediate assignment (only if other makefiles are somehow included)
mkfile_path := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

DEV_ARDUINO ?= /dev/arduino


build: Compiled/ArduinoSketch.ino.hex

accstepper: libraries/AccelStepper
	cd libraries/AccelStepper;\
	 	git fetch;\
	 	git diff-index --quiet HEAD -- || export LOCAL_CHANGES=1;\
	 	[ -z "$$LOCAL_CHANGES" ] || git stash;\
	 	git reset --hard origin/master;\
	 	[ -z "$$LOCAL_CHANGES" ] || git stash pop;

libraries/AccelStepper:
	mkdir -p libraries
	git clone git@github.com:momoson/AccelStepper.git libraries/AccelStepper

COMPILE_COMMAND = arduino-builder -hardware=/usr/share/arduino/hardware -tools=/usr/bin -fqbn=arduino:avr:uno -libraries libraries -verbose=true -build-path=$(mkfile_path)/Compiled fawkes_plugin_comm.ino

Compiled/ArduinoSketch.ino.hex: ArduinoSketch.ino accstepper
	mkdir -p Compiled
	$(COMPILE_COMMAND) || $(COMPILE_COMMAND) # sometimes compilation breaks, due to https://github.com/arduino/arduino-builder/issues/67. Repeating compilation normally resolves the compilation error

flash: Compiled/ArduinoSketch.ino.hex
	avrdude  -patmega328p -carduino -P$(DEV_ARDUINO) -B 10 -F -Uflash:w:$<:i

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

