#!/bin/bash
# Automated startup of a Gazebo simulation

usage()
{
cat << EOF
usage: $0 options

This script starts or kills the a Gazebo-simulation

OPTIONS:
   -h             Show this message
   -x start|kill  Start or kill simulation
   -c arg         Use a specific configuration-folder
                  in cfg/gazsim-configurations/
   -n arg         Specify number Robotinos
   -l             Run Gazebo headless
   -r             Start ros
   -k             Keep started shells open after finish
   -s             Keep statistics and shutdown after game
EOF
}
 
#check options

COMMAND=
CONF=
VISUALIZATION=
ROS=false
KEEP=
SHUTDOWN=
NUM_ROBOTINOS=3
while getopts “hx:c:lrksn:” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         c)
	     CONF=-c\ $OPTARG
             ;;
         l)
	     VISUALIZATION=-l
             ;;
         x)
	     COMMAND=$OPTARG
             ;;
         k)
	     KEEP=--profile\ refbox-shell 
             ;;
	 r)
	     ROS=-r
	     ;;
	 s)
	     SHUTDOWN=-s
	     ;;
	 n)
	     NUM_ROBOTINOS=$OPTARG
	     ;;
         ?)
             usage
             exit
             ;;
     esac
done

if [[ -z $COMMAND ]]
then
     usage
     exit 1
fi
if [ $NUM_ROBOTINOS -lt 1 ] || [ $NUM_ROBOTINOS -gt 3 ]
then
     echo Number Robotinos wrong
     exit 1
fi


#execute command

echo 'Automated Simulation control'

startup_script_location=~/fawkes-robotino/bin/gazsim-startup.bash 


if [  $COMMAND  == kill ]; then
    echo 'Kill Gazebo-sim'
    killall gazebo
    killall gzserver
    killall fawkes
    killall move_base
    killall roscore
    killall llsf-refbox
    killall llsf-refbox-shell
    exit 0
fi

if [  $COMMAND  == start ]; then
    #start gazebo
    
    gnome-terminal $KEEP -t Gazebo -x bash -c "$startup_script_location -x gazebo $VISUALIZATION"
    sleep 25s

    if [  $ROS  == "-r" ]; then
	echo 'Starting Ros'
        #start roscores
	gnome-terminal $KEEP -t Roscore1 -x bash -c "$startup_script_location -x roscore -p 11311"
	if [ $NUM_ROBOTINOS -ge 2 ]
	then
	    sleep 1s
	    gnome-terminal $KEEP -t Roscore2 -x bash -c "$startup_script_location -x roscore -p 11312"
	    if [ $NUM_ROBOTINOS -ge 3 ]
	    then
		sleep 1s
		gnome-terminal $KEEP -t Roscore3 -x bash -c "$startup_script_location -x roscore -p 11313"
	    fi
	fi
	
	sleep 2s #move_base quits if there is no roscore

        #start move_bases
	gnome-terminal $KEEP -t Move_base1 -x bash -c "$startup_script_location -x move_base -p 11311"
	if [ $NUM_ROBOTINOS -ge 2 ]
	then
	    gnome-terminal $KEEP -t Move_base2 -x bash -c "$startup_script_location -x move_base -p 11312"
	    if [ $NUM_ROBOTINOS -ge 3 ]
	    then
		gnome-terminal $KEEP -t Move_base3 -x bash -c "$startup_script_location -x move_base -p 11313"
	    fi
	fi
    fi

    #start refbox
    gnome-terminal $KEEP -t Refbox -x bash -c "$startup_script_location -x refbox"
    sleep 2s
    #start refbox shell
    gnome-terminal $KEEP -t Refbox_Shell --geometry=87x82 -x bash -c "$startup_script_location -x refbox-shell"

    #start fawkes for communication, llsfrbcomm and eventually statistics
    gnome-terminal $KEEP -t Fawkes_Comm -x bash -c "$startup_script_location -x comm -p 11311  $SHUTDOWN"

    #start fawkes for robotinos
    echo "$startup_script_location -x fawkes -p 11311 -i robotino1 $CONF $ROS"
    gnome-terminal $KEEP -t Fawkes_Robotino_1 -x bash -c "$startup_script_location -x fawkes -p 11311 -i robotino1 $CONF $ROS"
    if [ $NUM_ROBOTINOS -ge 2 ]
    then
	gnome-terminal $KEEP -t Fawkes_Robotino_2 -x bash -c "$startup_script_location -x fawkes -p 11312 -i robotino2 $CONF $ROS"
	if [ $NUM_ROBOTINOS -ge 3 ]
	then
	    gnome-terminal $KEEP -t Fawkes_Robotino_3 -x bash -c "$startup_script_location -x fawkes -p 11313 -i robotino3 $CONF $ROS"
	fi
    fi
    else
    usage
fi
