#!/bin/bash
# Automated startup of a Gazebo simulation

if [ ! $#  == 1 ] || ( [ ! $1  == start ] && [ ! $1  == start-ros ] && [ ! $1  == kill ] ); then
    echo 'Usage: gazsim.bash start|start-ros|kill'
    exit 0
fi

echo 'Automated Simulation startup'

startup_script_location=~/fawkes-robotino/src/tools/gazsim/gazsim-startup.bash 

if [  $1  == kill ]; then
    echo 'Kill Gazebo-sim'
    killall gazebo
    killall fawkes
    killall move_base
    killall roscore
    killall llsf-refbox
    killall llsf-refbox-shell
    exit 0
fi

if [  $1  == start-ros ]; then
    echo 'Starting Ros'
    #start roscores
    gnome-terminal -t Roscore1 -x bash -c "$startup_script_location roscore 11311"
    gnome-terminal -t Roscore2 -x bash -c "$startup_script_location roscore 11312"
    gnome-terminal -t Roscore3 -x bash -c "$startup_script_location roscore 11313"

    sleep 2s #move_base quits if there is no roscore

    #start move_bases
    gnome-terminal -t Move_base1 -x bash -c "$startup_script_location move_base 11311"
    gnome-terminal -t Move_base2 -x bash -c "$startup_script_location move_base 11312"
    gnome-terminal -t Move_base3 -x bash -c "$startup_script_location move_base 11313"
fi

#start gazebo
gnome-terminal -t Gazebo -x bash -c "$startup_script_location gazebo"


#start refbox
gnome-terminal -t Refbox -x bash -c "$startup_script_location refbox"
#start refbox shell
gnome-terminal -t Refbox_Shell --geometry=87x82 -x bash -c "$startup_script_location refbox_shell"

#start fawkes for communication and llsfrbcomm
gnome-terminal -t Fawkes_Comm -x bash -c "$startup_script_location fawkes 11311 comm"

#start fawkes for robotinos
if [  $1  == start-ros ]; then
    gnome-terminal -t Fawkes_Robotino_1 -x bash -c "$startup_script_location fawkes 11311 robotino1 ros"
    gnome-terminal -t Fawkes_Robotino_2 -x bash -c "$startup_script_location fawkes 11312 robotino2 ros"
    gnome-terminal -t Fawkes_Robotino_3 -x bash -c "$startup_script_location fawkes 11313 robotino3 ros"
else
    gnome-terminal -t Fawkes_Robotino_1 -x bash -c "$startup_script_location fawkes 11311 robotino1"
    gnome-terminal -t Fawkes_Robotino_2 -x bash -c "$startup_script_location fawkes 11312 robotino2"
    gnome-terminal -t Fawkes_Robotino_3 -x bash -c "$startup_script_location fawkes 11313 robotino3"
fi
