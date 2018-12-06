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
   --team-cyan       Set cyan team name
   --team-magenta		 Set magenta team name
   --start-game      Automatically run game after initialization
                     (if used with -t go into PRODUCTION phase,
                      otherwise the phase will be EXPLORATION,
                      optionally, "--start-game=PHASE" may be given
                      to set the phase explicitly)
                     Typically requires at least --team-cyan.
EOF
}

 
#check options

COMMAND=start
CONF=
HEADLESS=
ROS=
ROS_LAUNCH_MAIN=
ROS_LAUNCH_ROBOT=
AGENT=
DETAILED=
KEEP=
SHUTDOWN=
NUM_ROBOTINOS=3
NUM_CYAN=3
NUM_MAGENTA=0
FIRST_ROBOTINO_NUMBER=1
REPLAY=
FAWKES_BIN=$FAWKES_DIR/bin
META_PLUGIN=
START_GAZEBO=true
TERM_GEOMETRY=105x56
GDB=
SKIP_EXPLORATION=
FAWKES_USED=false
START_GAME=
TEAM_CYAN=
TEAM_MAGENTA=

if [ -z $TERMINAL ] ; then
    if [[ -n $TMUX ]] ; then
        TERMINAL=tmux
    else
        TERMINAL=gnome-terminal
    fi
fi

case "$TERMINAL" in
    gnome-terminal)
        TERM_COMMAND="gnome-terminal --window --geometry=$TERM_GEOMETRY -- bash -i -c '"
        TERM_COMMAND_END="'"
        SUBTERM_PREFIX="gnome-terminal --tab -- "
        SUBTERM_SUFFIX=" ; "
        ;;
    screen)
        TERM_COMMAND="screen -A -d -m -S gazsim /usr/bin/sleep 1 ; "
        SUBTERM_PREFIX="screen -S gazsim -X screen "
        SUBTERM_SUFFIX=" ; "
        ;;
    tmux)
        if [[ -n $TMUX ]] ; then
            TERM_COMMAND=""
        else
            TERM_COMMAND="tmux new-session -d;"
        fi
        TERM_COMMAND_END=""
        SUBTERM_PREFIX="tmux new-window "
        SUBTERM_SUFFIX=";"
        ;;
    *)
        >&2 echo "Unknown terminal $TERMINAL"
        exit 1
esac

echo "Using $TERMINAL"

ROS_MASTER_PORT=${ROS_MASTER_URI##*:}
ROS_MASTER_PORT=${ROS_MASTER_PORT%%/*}

OPTS=$(getopt -o "hx:c:lrksn:e:dm:aof:p:gvt" -l "ros,ros-launch-main:,ros-launch:,start-game::,team-cyan:,team-magenta:" -- "$@")
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
	     HEADLESS=-l
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
	     META_PLUGIN="-m gazsim-meta-clips-exec"
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
	 --team-cyan)
	     TEAM_CYAN="$OPTARG"
	     ;;
	 --team-magenta)
	     TEAM_MAGENTA="$OPTARG"
	     ;;
	 --start-game)
			 if [ -n "$OPTARG" ]; then
					 START_GAME="$OPTARG"
			 else
					 if [ -n "$SKIP_EXPLORATION" ]; then
							 START_GAME="PRODUCTION"
					 else
							 START_GAME="EXPLORATION"
					 fi
			 fi
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
	 --)
	     shift
	     break
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


if [ $FIRST_ROBOTINO_NUMBER -le 0 ] || [ $FIRST_ROBOTINO_NUMBER -gt 6 ]
then
     echo Invalid first robotino number, must be in 1..6.
     exit 1
fi
if [ $NUM_ROBOTINOS -le 0 ] || [ $(($FIRST_ROBOTINO_NUMBER-1+$NUM_ROBOTINOS)) -gt 6 ]
then
     echo Invalid number of robotinos, must be 1..$((6-$FIRST_ROBOTINO_NUMBER+1))
     exit 1
fi

if [ $FIRST_ROBOTINO_NUMBER -gt 3 ]; then
  NUM_CYAN=0
else
  NUM_CYAN=$((3 - $FIRST_ROBOTINO_NUMBER + 1))
  if [ $NUM_CYAN -gt $NUM_ROBOTINOS ]; then
    NUM_CYAN=$NUM_ROBOTINOS
  fi
fi
NUM_MAGENTA=$(($NUM_ROBOTINOS-$NUM_CYAN))

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
    [[ -f /dev/shm/*fawkes* ]] && rm /dev/shm/*fawkes*

    #construct command to open everything in one terminal window with multiple tabs instead of 10.000 windows

    OPEN_COMMAND="$TERM_COMMAND"
    COMMANDS=()

    if $START_GAZEBO
    then
	#start gazebo
	if [[ -z $HEADLESS ]]
	then
        COMMANDS+=("bash -i -c \"$startup_script_location -x gazebo $REPLAY $KEEP $@\"")
	else
	    #run headless
        COMMANDS+=("bash -i -c \"$startup_script_location -x gzserver $REPLAY $KEEP $@\"")
	fi
    fi

    if [  "$ROS"  == "-r" ]; then
    	#start roscores
	# main roscore (non-robot)
    COMMANDS+=("bash -i -c \"$startup_script_location -x roscore -p $ROS_MASTER_PORT $KEEP $@\"")
	if [ -n "$ROS_LAUNCH_MAIN" ]; then
		COMMANDS+=("bash -i -c \"$startup_script_location -x roslaunch $ROS_LAUNCH_MAIN -p $ROS_MASTER_PORT $KEEP $@\"")
	fi
    	for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    	do
	    # robot roscore
	    COMMANDS+=("bash -i -c \"$startup_script_location -x roscore -p 1132$ROBO $KEEP $@\"")
            # move_base
	    COMMANDS+=("bash -i -c \"$startup_script_location -x move_base -p 1132$ROBO $KEEP $@\"")
	if [ -n "$ROS_LAUNCH_ROBOT" ]; then
	    COMMANDS+=("bash -i -c \"$startup_script_location -x roslaunch $ROS_LAUNCH_ROBOT -p $ROS_MASTER_PORT $KEEP $@\"")
	fi
    	done
    fi

    if $START_GAZEBO
    then
	    #start refbox
	    COMMANDS+=("bash -i -c \"$startup_script_location -x refbox $KEEP $@\"")
    	#start refbox shell
        COMMANDS+=("bash -i -c \"$startup_script_location -x refbox-shell $KEEP $@\"")
    fi

    #start fawkes for robotinos
    for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    do
	COMMANDS+=("bash -i -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 10; $startup_script_location -x fawkes -p 1132$ROBO -i robotino$ROBO $KEEP $CONF $ROS $ROS_LAUNCH_MAIN $ROS_LAUNCH_ROBOT $GDB $META_PLUGIN $DETAILED -f $FAWKES_BIN $SKIP_EXPLORATION $@\"")
	FAWKES_USED=true
    done

    if $START_GAZEBO
    then
    	#start fawkes for communication, llsfrbcomm and eventually statistics
	COMMANDS+=("bash -i -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 5; $startup_script_location -x comm $KEEP $SHUTDOWN $@\"")
    fi

    PREFIXED_COMMANDS=("${COMMANDS[@]/#/${SUBTERM_PREFIX}}")
    SUFFIXED_COMMANDS=("${PREFIXED_COMMANDS[@]/%/${SUBTERM_SUFFIX}}")
    echo "Executing $TERM_COMMAND ${SUFFIXED_COMMANDS[@]} ${TERM_COMMAND_END}"
    eval "$TERM_COMMAND ${SUFFIXED_COMMANDS[@]} ${TERM_COMMAND_END}"

    if $FAWKES_USED
    then
	# publish initial poses
	echo "publish initial poses"
	$initial_pose_script_location -c $NUM_CYAN -m $NUM_MAGENTA -d
    else
	echo "Skipped publishing poses"
  sleep 10
    fi

		if [ -n "$START_GAME" ]; then
				if [ ! -x $LLSF_REFBOX_DIR/bin/rcll-refbox-instruct ]; then
						echo "rcll-refbox-instruct not found, not built or old version?"
				else
						echo "Starting game (Phase: $START_GAME ${TEAM_CYAN:+Cyan: ${TEAM_CYAN}}${TEAM_MAGENTA:+ Magenta: ${TEAM_MAGENTA}})"
						$LLSF_REFBOX_DIR/bin/rcll-refbox-instruct -p $START_GAME -s RUNNING ${TEAM_CYAN:+-c ${TEAM_CYAN}}${TEAM_MAGENTA:+-m ${TEAM_MAGENTA}}
				fi
		fi

    else
    usage
fi

# vim:et:sw=4:ts=4
