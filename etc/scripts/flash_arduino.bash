#!/bin/bash
# Helper script for flashing the arduino
# args: 1. path to ArduinoSketch (in Arduino plugin)

($1/ArduinoSketch verify &> /dev/null) \
&& echo -e "--> No need to reflash arduino" \
|| ($1/ArduinoSketch flash &> /dev/null) \
&& echo -e "--> Successfully reflashed arduino" \
echo -e "--> Something went wrong while flashing the arduino"; exit 1

