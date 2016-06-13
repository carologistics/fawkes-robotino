#!/bin/bash
# Automated startup of a Gazebo simulation

usage()
{
cat << EOF
usage: $0 options

This script starts or kills the a Gazebo-simulation

OPTIONS:
   -h                Show this message
   -x start|kill     Start or kill simulation
   -c arg            Use a specific configuration-folder
                     in cfg/gazsim-configurations/
   -n arg            Specify number Robotinos
   -m arg            load fawkes with the specified (meta-)plugin
   -a                Run with default CLIPS-agent (don't mix with -m)
   -l                Run Gazebo headless
   -k                Keep started shells open after finish
   -s                Keep statistics and shutdown after game
   -r|--ros          Start with ROS support
   --ros-launch-main Run ROS launch file once for main (non-robot) master
                     Argument: package:file.launch
                     Calls: roslaunch package file.launch
   --ros-launch      Run launch file for each robot (on their roscore)
   -e arg            Record replay
   -d                Detailed simulation (e.g. simulated webcam)
   -o                Omitt starting gazebo (necessary when starting
                     different teams)
   -f arg            First Robotino Number (default 1, choose 4 when
                     starting as magenta)
   -p arg            Path to the fawkes folder
                     ($FAWKES_DIR/bin by default)
   -g                Run Fawkes in gdb
   -v                Run Fawkes in valgrind
   -t                Skip Exploration and add all navgraph points
EOF
}

 
#check options

COMMAND=start
CONF=
VISUALIZATION=
ROS=
ROS_LAUNCH_MAIN=
ROS_LAUNCH_ROBOT=
AGENT=
DETAILED=
KEEP=
SHUTDOWN=
NUM_ROBOTINOS=3
FIRST_ROBOTINO_NUMBER=1
REPLAY=
FAWKES_BIN=$FAWKES_DIR/bin
META_PLUGIN=
START_GAZEBO=true
TERM_GEOMETRY=105x56
GDB=
SKIP_EXPLORATION=
FAWKES_USED=false

OPTS=$(getopt -o "hx:c:lrksn:e:dm:aof:p:gvt" -l "ros,ros-launch-main:,ros-launch:" -- "$@")
if [ $? != 0 ]
then
    echo "Failed to parse parameters"
    usage
    exit 1
fi

eval set -- "$OPTS"
while true; do
     OPTION=$1
     OPTARG=$2
     case $OPTION in
         -h)
    	     echo "Display help on request"
             usage
             exit 1
             ;;
         -c)
	     CONF=-c\ $OPTARG
             ;;
         -l)
	     VISUALIZATION=-l
             ;;
         -x)
	     COMMAND=$OPTARG
             ;;
         -k)
	     KEEP=-k
             ;;
         -r)
	     ROS=-r
             ;;
         -g)
	     if [ -n "$GDB" ]; then
				echo "Can pass only either valgrind or GDB, not both"
        exit
       fi
	     GDB=-g
             ;;
         -v)
	     if [ -n "$GDB" ]; then
				echo "Can pass only either valgrind or GDB, not both"
        exit
       fi
	     GDB=-v
             ;;
	 -s)
	     SHUTDOWN=-s
	     #done in two steps because otherwise ps would find grep serching for the pattern
	     PS=$(ps -ef)
	     if [[ -z $(echo $PS | grep -i 'mongod') ]]
	     then
		 echo Please start the mongodb service \(sudo service mongod start\)
		 exit
	     fi
	     ;;
	 -n)
	     NUM_ROBOTINOS=$OPTARG
	     ;;
	 -e)
	     REPLAY="-e $OPTARG"
	     ;;
	 -d)
	     DETAILED="-d"
	     ;;
	 -m)
	     META_PLUGIN="-m $OPTARG"
	     ;;
	 -a)
	     META_PLUGIN="-m gazsim-meta-agent"
	     ;;
	 -o)
	     START_GAZEBO=false
	     ;;
	 -r|--ros)
	     ROS=-r
	     ;;
	 --ros-launch-main)
	     ROS_LAUNCH_MAIN="--ros-launch $OPTARG"
	     ;;
	 --ros-launch-robot)
	     ROS_LAUNCH_ROBOT="--ros-launch $OPTARG"
	     ;;
	 -f)
	     FIRST_ROBOTINO_NUMBER=$OPTARG
	     ;;
	 -t)
	     SKIP_EXPLORATION="-t"
	     ;;
	 -p)
	     FAWKES_BIN=$OPTARG/bin
	     ;;
	 --) break;
             ;;
     esac
     shift
done

if [[ -z $COMMAND ]]
then
     echo "No command given"
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

    OPEN_COMMAND="gnome-terminal --geometry=$TERM_GEOMETRY"

    if $START_GAZEBO
    then
	#start gazebo
	if [[ -z $VISUALIZATION ]]
	then
	    OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x gazebo $REPLAY $KEEP\"'"
	else
	    #run headless
	    OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x gzserver $REPLAY $KEEP\"'"
	fi
    fi

    if [  "$ROS"  == "-r" ]; then
    	#start roscores
	# main roscore (non-robot)
	OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x roscore -p ${ROS_MASTER_URI##*:} $KEEP\"'"
	if [ -n "$ROS_LAUNCH_MAIN" ]; then
		OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x roslaunch $ROS_LAUNCH_MAIN -p ${ROS_MASTER_URI##*:} $KEEP\"'"
	fi
    	for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    	do
	    # robot roscore
	    OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x roscore -p 1132$ROBO $KEEP\"'"
	if [ -n "$ROS_LAUNCH_ROBOT" ]; then
		OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x roslaunch $ROS_LAUNCH_ROBOT -p $ROS_MASTER_URI $KEEP\"'"
	fi
    	done
    fi

    if $START_GAZEBO
    then
	#start refbox
	OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x refbox $KEEP\"'"
    	#start refbox shell
    	OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"$startup_script_location -x refbox-shell $KEEP\"'"
    fi

    #start fawkes for robotinos
    for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    do
	OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 10; $startup_script_location -x fawkes -p 1132$ROBO -i robotino$ROBO $KEEP $CONF $ROS $ROS_LAUNCH_MAIN $ROS_LAUNCH_ROBOT $GDB $META_PLUGIN $DETAILED -f $FAWKES_BIN $SKIP_EXPLORATION\"'"
	FAWKES_USED=true
    done

    if $START_GAZEBO
    then
    	#start fawkes for communication, llsfrbcomm and eventually statistics
	OPEN_COMMAND="$OPEN_COMMAND --tab -e 'bash -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 5; $startup_script_location -x comm $KEEP $SHUTDOWN\"'"
    fi

    # open windows
    #echo $OPEN_COMMAND
    eval $OPEN_COMMAND

    sleep 10s
    if $FAWKES_USED
    then
	sleep 15s
	# publish initial poses
	echo "publish initial poses"
	$initial_pose_script_location -d
    else
	echo "Skipped publishing poses"
    fi


    else
    usage
fi
