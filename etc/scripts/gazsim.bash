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
   -m arg         load fawkes with the specified (meta-)plugin
   -a             Run with default CLIPS-agent (don't mix with -m)
   -l             Run Gazebo headless
   -k             Keep started shells open after finish
   -s             Keep statistics and shutdown after game
   -r             Start with ROS
   -e arg         Record replay
   -d             Detailed simulation (e.g. simulated webcam)
   -o             Omitt starting gazebo (necessary when starting
                  different teams)
   -f arg         First Robotino Number (default 1, choose 4 when
                  starting as magenta)
   -p arg         Path to the fawkes folder
                  ($FAWKES_DIR/bin by default)
EOF
}
 
#check options

COMMAND=
CONF=
VISUALIZATION=
ROS=
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
while getopts “hx:c:lrksn:e:dm:aof:p:” OPTION
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
	     META_PLUGIN="-m gazsim-meta-agent"
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

    #construct command to open everything in one terminal window with multiple tabs instead of 10.000 windows
    OPEN_COMMAND="gnome-terminal"

    if $START_GAZEBO
    then
	#start gazebo
	if [[ -z $VISUALIZATION ]]
	then
	    OPEN_COMMAND="$OPEN_COMMAND --tab -t Gazebo -e 'bash -c \"$startup_script_location -x gazebo $REPLAY $KEEP\"'"
	else
	    #run headless
	    OPEN_COMMAND="$OPEN_COMMAND --tab -t Gzserver -e 'bash -c \"$startup_script_location -x gzserver $REPLAY $KEEP\"'"
	fi
    fi

    if [  $ROS  == "-r" ]; then
    	#start roscores
    	for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    	do
	    OPEN_COMMAND="$OPEN_COMMAND --tab -t Roscore$ROBO -e 'bash -c \"$startup_script_location -x roscore -p 1131$ROBO $KEEP\"'"
    	done
    fi

    if $START_GAZEBO
    then
	#start refbox
	OPEN_COMMAND="$OPEN_COMMAND --tab -t Refbox -e 'bash -c \"$startup_script_location -x refbox $KEEP\"'"
    	#start refbox shell
    	OPEN_COMMAND="$OPEN_COMMAND --tab --geometry=87x82 -t Refbox_Shell -e 'bash -c \"$startup_script_location -x refbox-shell $KEEP\"'"
    fi

    #start fawkes for robotinos
    for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    do
	OPEN_COMMAND="$OPEN_COMMAND --tab -t Fawkes_Robotino_$ROBO -e 'bash -c \"sleep 10s; $startup_script_location -x fawkes -p 1131$ROBO -i robotino$ROBO $KEEP $CONF $ROS $META_PLUGIN $DETAILED -f $FAWKES_BIN\"'"
    done

    if $START_GAZEBO
    then
    	#start fawkes for communication, llsfrbcomm and eventually statistics
	OPEN_COMMAND="$OPEN_COMMAND --tab -t Fawkes_Comm -e 'bash -c \"sleep 5s ; $startup_script_location -x comm -p 11311 $KEEP $SHUTDOWN\"'"
    fi

    # open windows
    #echo $OPEN_COMMAND
    eval $OPEN_COMMAND

    # publish initial poses
    sleep 15s
    echo "publish initial poses"
    $initial_pose_script_location -d

    else
    usage
fi
