.PHONY: build flash clean

build: Compiled/fawkes_plugin_comm.ino.hex

Compiled/fawkes_plugin_comm.ino.hex: fawkes_plugin_comm.ino
	mkdir -p Compiled
	arduino-builder -hardware=/usr/share/arduino/hardware -tools=/usr/bin -fqbn=arduino:avr:uno -libraries /home/$(shell whoami)/Arduino/libraries -verbose=true -build-path=/home/momo/fawkes-robotino/src/plugins/arduino/ArduinoSketch/Compiled fawkes_plugin_comm.ino

flash: Compiled/fawkes_plugin_comm.ino.hex build
	avrdude  -patmega328p -carduino -P/dev/arduino -B 10 -F -Uflash:w:$<:i

clean:
	rm -rf Compiled
