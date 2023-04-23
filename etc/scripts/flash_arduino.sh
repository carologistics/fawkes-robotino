#!/bin/sh

# Helper script for flashing the arduino
# args: 1. path to ArduinoSketch (in Arduino plugin)
# args: 2. path to the Arduino (i.e. /dev/arduino on the Robotinos)

compile_command="arduino-cli compile -b arduino:avr:uno $1/ArduinoSketch/ --build-path $1/ArduinoSketch/Compiled"
check_command="avrdude -c arduino -p ATmega328P -P $2 -B 10 -F -U flash:v:$1/ArduinoSketch/Compiled/ArduinoSketch.ino.hex"
flash_command="arduino-cli upload -b arduino:avr:uno $1/ArduinoSketch --input-dir $1/ArduinoSketch/Compiled --port $2 --log-level debug -v"
#($1/ArduinoSketch verify &> /dev/null) \
#&& echo -e "--> No need to reflash arduino" \
#|| ($1/ArduinoSketch flash &> /dev/null) \
#&& echo -e "--> Successfully reflashed arduino" \
#echo -e "--> Something went wrong while flashing the arduino"; exit 1

if [ -e $2 ]
then
	echo -e "\e[32mArduino device exists: $2\e[0m"
else
	echo -e "\e[31mArduino device does not exist: $2\n" + \
	"Set the DEV_ARDUINO cmake variable to the arduino device!\e[0m"
fi

eval $compile_command > /dev/null 2>&1
if [[ $? == 0 ]]
then
 	echo -e "\e[32mSuccesfully compiled the arduino sketch!\e[0m"
else
 	echo -e "\e[31mFailed to compiled the arduino sketch!\e[0m\n" \
	"Used the Command: $compile_command"
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
	fi
fi