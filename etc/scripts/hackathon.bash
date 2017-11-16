#!/bin/bash
# Automated startup of a Gazebo simulation

usage()
{
cat << EOF
usage: $0 options

This script starts or kills the a Gazebo-simulation

OPTIONS:
   -h             Show this message
   -k             Keep started shells open after finish
EOF
}

 
#check options

COMMAND=start
CONF=
VISUALIZATION=
ROS=-r
AGENT=
DETAILED=-d
KEEP=
SHUTDOWN=
NUM_ROBOTINOS=1
FIRST_ROBOTINO_NUMBER=1
REPLAY=
FAWKES_BIN=$FAWKES_DIR/bin
META_PLUGIN=
START_GAZEBO=true
GAZEBO_WORLD=${GAZEBO_WORLD_PATH:-~/robotics/gazebo-rcll/worlds/carologistics/hackathon2017.world}
while getopts “hx:c:lrksn:e:dm:aof:p:w” OPTION
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
	     KEEP=-k
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
	 m)
	     META_PLUGIN="-m $OPTARG"
	     ;;
	 a)
	     META_PLUGIN="-m gazsim-meta-clips-exec"
	     ;;
	 o)
	     START_GAZEBO=false
	     ;;
	 r)
	     ROS=-r
	     ;;
	 f)
	     FIRST_ROBOTINO_NUMBER=$OPTARG
	     ;;
	 p)
	     FAWKES_BIN=$OPTARG/bin
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

if [ $NUM_ROBOTINOS -lt 0 ] || [ $NUM_ROBOTINOS -gt 6 ]
then
     echo Number Robotinos wrong
     exit 1
fi


#execute command

echo 'Automated Simulation control'

script_path=$FAWKES_DIR/bin
startup_script_location=$script_path/gazsim-startup.bash 
initial_pose_script_location=$script_path/gazsim-publish-initial-pose.bash 

if [  $COMMAND  == kill ]; then
    echo 'Kill Gazebo-sim'
    #killall gazebo
    killall gzserver
    killall gzclient
    killall fawkes
    killall roscore
    killall llsf-refbox
    killall llsf-refbox-shell
    killall roslaunch
    exit 0
fi

if [  $COMMAND  == start ]; then

    #check if enviromnental variables for gazebo are set
    if [ -z "$FAWKES_DIR" ]; then
	echo "FAWKES_DIR is not set"
	exit 1
    fi
    if ! [[ $GAZEBO_PLUGIN_PATH == *lib/gazebo* ]]
    then
	echo "Missing path to Gazebo Plugins in GAZEBO_PLUGIN_PATH";
	exit 1
    fi

    # delete old shm files. Only do this for the simulation, not on live bot.
    rm /dev/shm/*fawkes*

    #construct command to open everything in one terminal window with multiple tabs instead of 10.000 windows

    OPEN_COMMAND="gnome-terminal"

    if $START_GAZEBO
    then
	#start gazebo
	if [[ -z $VISUALIZATION ]]
	then
	    OPEN_COMMAND="$OPEN_COMMAND --tab -t Gazebo -e 'bash -c \"$startup_script_location -x gazebo $REPLAY $KEEP -w $GAZEBO_WORLD\"'"
	else
	    #run headless
	    OPEN_COMMAND="$OPEN_COMMAND --tab -t Gzserver -e 'bash -c \"$startup_script_location -x gzserver $REPLAY $KEEP -w $GAZEBO_WORLD\"'"
	fi
    fi

    if [  $ROS  == "-r" ]; then
    	#start roscores
    	for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    	do
	    OPEN_COMMAND="$OPEN_COMMAND --tab -t Roscore$ROBO -e 'bash -c \"$startup_script_location -x roscore -p 1131$ROBO $KEEP\"'"
    	done
    fi

    #start fawkes for robotinos
    for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    do
	OPEN_COMMAND="$OPEN_COMMAND --tab -t Fawkes_Robotino_$ROBO -e 'bash -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 10; $startup_script_location -x fawkes -p 1131$ROBO -i robotino$ROBO $KEEP $CONF $ROS $META_PLUGIN $DETAILED -f $FAWKES_BIN\"'"
    done

    if $START_GAZEBO
    then
    	#start fawkes for communication, llsfrbcomm and eventually statistics
	OPEN_COMMAND="$OPEN_COMMAND --tab -t Fawkes_Comm -e 'bash -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 5; $startup_script_location -x comm -p 11311 $KEEP $SHUTDOWN\"'"
    fi

    if $START_MOVE_BASE
    then
	#start move_base for path planning
	OPEN_COMMAND="$OPEN_COMMAND --tab -t move_base -e 'bash -c \"roslaunch --wait robotino_move_base robotino_move_base_simu.launch\"'"
    fi

    # open windows
    #echo $OPEN_COMMAND
    eval $OPEN_COMMAND

    # publish initial poses
    sleep 15s
    echo "publish initial poses"
    $initial_pose_script_location -p 1921 -x 0.7 -y 2.461 -o 0 0 0.7 0.7

    else
    usage
fi
