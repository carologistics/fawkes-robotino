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
   -e arg         Record replay
   -a             Start with agent
   -d             Detailed simulation (e.g. simulated webcam)
EOF
}
 
#check options

COMMAND=
CONF=
VISUALIZATION=
ROS=false
AGENT=
DETAILED=
KEEP=
SHUTDOWN=
NUM_ROBOTINOS=3
REPLAY=
while getopts “hx:c:lrksn:e:da” OPTION
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
	     #done in two steps because otherwise ps would find grep serching for the pattern
	     PS=$(ps -ef)
	     if [[ -z $(echo $PS | grep -i 'mongod') ]]
	     then
		 echo Please start the mongodb service \(sudo service mongod start\)
		 exit
	     fi
	     ;;
	 n)
	     NUM_ROBOTINOS=$OPTARG
	     ;;
	 e)
	     REPLAY="-e $OPTARG"
	     ;;
	 d)
	     DETAILED="-d"
	     ;;
	 a)
	     AGENT="-a"
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

script_path=$FAWKES_BIN
startup_script_location=$script_path/gazsim-startup.bash 
initial_pose_script_location=$script_path/gazsim-publish-initial-pose.bash 

if [  $COMMAND  == kill ]; then
    echo 'Kill Gazebo-sim'
    killall gazebo
    killall gzserver
    killall gzclient
    killall fawkes
    killall move_base
    killall roscore
    killall llsf-refbox
    killall llsf-refbox-shell
    exit 0
fi

if [  $COMMAND  == start ]; then

    #check if enviromnental variables for gazebo are set
    if ! [[ $GAZEBO_MODEL_PATH == *fawkes-robotino/res/gazebo-models* ]]
    then
	echo "Missing path to Gazebo Models in GAZEBO_MODEL_PATH";
	exit 0
    fi
    if ! [[ $GAZEBO_PLUGIN_PATH == *fawkes-robotino/lib/gazebo* ]]
    then
	echo "Missing path to Gazebo Plugins in GAZEBO_PLUGIN_PATH";
	exit 0
    fi
    if ! [[ $FAWKES_BIN == *fawkes-robotino/bin* ]]
    then
	echo "Missing path to fawkes-robotino/bin in FAWKES_BIN";
	exit 0
    fi

    #start gazebo
    #server
    gnome-terminal $KEEP -t Gzserver -x bash -c "$startup_script_location -x gzserver $REPLAY"
    #client if not headless
    if [[ -z $VISUALIZATION ]]
    then
        gnome-terminal $KEEP -t Gzclient -x bash -c "$startup_script_location -x gzclient"
    fi
    sleep 25s

    if [  $ROS  == "-r" ]; then
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

    #start fawkes for robotinos
    gnome-terminal $KEEP -t Fawkes_Robotino_1 -x bash -c "$startup_script_location -x fawkes -p 11311 -i robotino1 $CONF $ROS $AGENT $DETAILED"
    if [ $NUM_ROBOTINOS -ge 2 ]
    then
	gnome-terminal $KEEP -t Fawkes_Robotino_2 -x bash -c "$startup_script_location -x fawkes -p 11312 -i robotino2 $CONF $ROS $AGENT $DETAILED"
	if [ $NUM_ROBOTINOS -ge 3 ]
	then
	    gnome-terminal $KEEP -t Fawkes_Robotino_3 -x bash -c "$startup_script_location -x fawkes -p 11313 -i robotino3 $CONF $ROS $AGENT $DETAILED"
	fi
    fi

    sleep 5s

    #start fawkes for communication, llsfrbcomm and eventually statistics
    gnome-terminal $KEEP -t Fawkes_Comm -x bash -c "$startup_script_location -x comm -p 11311  $SHUTDOWN"

    sleep 1s

    # publish initial poses
    $initial_pose_script_location -d

    else
    usage
fi
