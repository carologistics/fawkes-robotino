#!/bin/bash
# function to wait only the first time the terminal tab started
# and to restart immediately if it was restarted

#args 1: sleep time

NOW=$(date +%d%H%M%S)

TIME_DIFF=$(($NOW-$TAB_START_TIME))

if [ $TIME_DIFF -lt 10 ]
    then
    # sleep in the first 10 seconds of the tab
    sleep $1s
fi
#otherwise start fawkes directly


