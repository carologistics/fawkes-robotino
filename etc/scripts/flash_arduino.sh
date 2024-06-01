#!/bin/sh

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.


# Helper script for flashing the arduino
# args: 1. path to ArduinoSketch (in Arduino plugin)
# args: 2. path to the Arduino (i.e. /dev/arduino on the Robotinos)

compile_command="arduino-cli compile -b arduino:avr:uno $1/ArduinoSketch/ --build-path $1/ArduinoSketch/Compiled"
check_command="avrdude -c arduino -p ATmega328P -P $2 -B 10 -F -U flash:v:$1/ArduinoSketch/Compiled/ArduinoSketch.ino.hex"
flash_command="avrdude -p atmega328p -carduino -P $2 -B 10 -F -U flash:w:$1/ArduinoSketch/Compiled/ArduinoSketch.ino.hex:i"

if [ -e $2 ]
then
	echo -e "\e[32mArduino device exists: $2\e[0m"
else
	echo -e "\e[31mArduino device does not exist: $2\n" + \
	"Set the DEV_ARDUINO cmake variable to the arduino device!\e[0m"
	exit 0
fi

eval $compile_command > /dev/null 2>&1
if [[ $? == 0 ]]
then
 	echo -e "\e[32mSuccesfully compiled the arduino sketch!\e[0m"
else
 	echo -e "\e[31mFailed to compiled the arduino sketch!\e[0m\n" \
	"Used the Command: $compile_command"
	exit 2
fi

eval $check_command > /dev/null 2>&1

if [[ $? == 0 ]]
then
	echo -e "\e[32mNo need to flash the arduino!\e[0m"
else
	eval $flash_command > /dev/null 2>&1
	if [[ $? == 0 ]]
	then
		echo -e "\e[32mSuccesfully flashed the arduino!\e[0m"
	else
		echo -e "\e[31mFailed to flash the arduino!\e[0m\n" \
		"Used the Command: $flash_command"
		exit 3
	fi
fi
