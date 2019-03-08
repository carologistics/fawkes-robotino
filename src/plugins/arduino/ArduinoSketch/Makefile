.PHONY: build flash clean accstepper cleanall

# important immediate assignment (only if other makefiles are somehow included)
mkfile_path := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

build: accstepper Compiled/fawkes_plugin_comm.ino.hex

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

Compiled/fawkes_plugin_comm.ino.hex: fawkes_plugin_comm.ino
	mkdir -p Compiled
	arduino-builder -hardware=/usr/share/arduino/hardware -tools=/usr/bin -fqbn=arduino:avr:uno -libraries libraries -verbose=true -build-path=$(mkfile_path)/Compiled fawkes_plugin_comm.ino

flash: Compiled/fawkes_plugin_comm.ino.hex
	avrdude  -patmega328p -carduino -P/dev/arduino -B 10 -F -Uflash:w:$<:i

clean:
	rm -rf Compiled

cleanall: clean
	rm -rf libraries
