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
   --central-agent p Run an additional fawkes instance with plugin p
   -l                Run Gazebo headless
   -k                Keep started shells open after finish
   -s                Keep statistics and shutdown after game
   -r|--ros          Start with ROS support
   --ros-launch-main Run ROS launch file once for main (non-robot) master
                     Argument: package:file.launch
                     Calls: roslaunch package file.launch
   --ros-launch      Run launch file for each robot (on their roscore)
   -e arg            Record replay
   -d|--debug        Run Fawkes with debug output enabled
   --detailed        Detailed simulation (e.g. simulated webcam)
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
   --mongodb         Start central mongodb instance
   --keep-tmpfiles   Do not delete tmp files on exit
   --asp             Run with ASP agent and global planner
   --challenge       Start refbox challenge script instead of refbox
   --refbox-args     Pass options to the refbox
EOF
}


#check options

COMMAND=start
CONF=
HEADLESS=
REFBOX=refbox
REFBOX_ARGS=
ROS=
ROS_LAUNCH_MAIN=
ROS_LAUNCH_ROBOT=
ROS_LAUNCH_MOVEBASE=
AGENT=
DEBUG=
DETAILED=
KEEP=
KEEP_TMPFILES=
SHUTDOWN=
NUM_ROBOTINOS=3
NUM_CYAN=3
NUM_MAGENTA=0
FIRST_ROBOTINO_NUMBER=1
REPLAY=
FAWKES_BIN=$FAWKES_DIR/bin
META_PLUGIN=
CENTRAL_AGENT=
START_GAZEBO=true
GDB=
SKIP_EXPLORATION=
FAWKES_USED=false
START_GAME=
TEAM_CYAN=
TEAM_MAGENTA=
START_CENTRAL_AGENT=false
START_ASP_PLANER=false
START_MONGODB=false

if [ -z $TERMINAL ] ; then
    if [[ -n $TMUX ]] ; then
        TERMINAL=tmux
    else
        TERMINAL=gnome-terminal
    fi
fi

if [ -n $LLSF_REFBOX_DIR ] ; then
    export PATH=$LLSF_REFBOX_DIR/bin:$PATH
fi

case "$TERMINAL" in
    gnome-terminal)
        TERM_COMMAND="gnome-terminal --maximize -- bash -i -c '"
        TERM_COMMAND_END=" echo -e \"\n\n\nAll commands started. This tab may now be closed.\"'"
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
            TERM_COMMAND="tmux new-session -s gazsim -d;"
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

OPTS=$(getopt -o "hx:c:lrksn:e:dm:aof:p:gvt" -l "debug,ros,ros-launch-main:,ros-launch:,start-game::,team-cyan:,team-magenta:,mongodb,asp,central-agent:,keep-tmpfiles,challenge,refbox-args:" -- "$@")
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
        if [ "$TERMINAL" == "tmux" ] ; then
            TERM_COMMAND="$TERM_COMMAND tmux set-window-option -g remain-on-exit on;"
        fi
             ;;
         -r)
           ROS=-r
           ROS_LAUNCH_MOVE_BASE=yes
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
     -d|--debug)
         DEBUG="--debug"
         ;;
	 --detailed)
	     DETAILED="-d"
	     ;;
	 -m)
	     META_PLUGIN="-m $OPTARG"
	     ;;
	 -a)
	     META_PLUGIN="-m gazsim-meta-distributed-clips-exec"
	     ;;
     --mongodb)
         START_MONGODB=true
         ;;
     --challenge)
         REFBOX=refbox-challenge
         ;;
     --refbox-args)
         REFBOX_ARGS="$OPTARG"
         ;;
     --keep-tmpfiles)
         KEEP_TMPFILES=true
         ;;
	 --central-agent)
	     CENTRAL_AGENT="$OPTARG"
         START_CENTRAL_AGENT=true
	     ;;
	 --asp)
	     CONF="-c asp-planner"
	     META_PLUGIN="-m asp-sim-2016"
	     START_ASP_PLANER=true
       ROS_LAUNCH_MOVE_BASE=
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

function stop_simulation {
  echo 'Kill Gazebo-sim'
  if [ "$TERMINAL" == "tmux" ] ; then
      tmux kill-session -t gazsim
  else
      #killall gazebo
      killall gzserver
      killall gzclient
      killall fawkes
      killall roscore
      killall llsf-refbox
      killall roslaunch
  fi
}

if [  $COMMAND  == kill ]; then
  stop_simulation
  exit 0
fi

if [  $COMMAND  == start ]; then

    #check if enviromnental variables for gazebo are set
    if [ -z "$FAWKES_DIR" ]; then
	echo "FAWKES_DIR is not set"
	exit 1
    fi

    # delete old shm files. Only do this for the simulation, not on live bot.
    [[ -f /dev/shm/*fawkes* ]] && rm /dev/shm/*fawkes*

    # Re-define TMPDIR so we can delete all temporary files afterwards.
    GAZSIM_TMPDIR=$(mktemp -d --tmpdir gazsim-XXXXXXXXXXXX)
    TMPDIR="${GAZSIM_TMPDIR}"
    export TMPDIR

    #construct command to open everything in one terminal window with multiple tabs instead of 10.000 windows
    COMMANDS=()

    #if [[ -z "$KEEP_TMPFILES" ]] ; then
      #COMMANDS=("bash -i -c \"trap \\\"sleep 1; rm -rf $GAZSIM_TMPDIR\\\" EXIT; echo \\\"Cleanup on $GAZSIM_TMPDIR: Waiting for shutdown\\\"; while true; do sleep 1; done\"")
    #fi

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
				if $START_GAZEBO && [ -n "$ROS_LAUNCH_MOVE_BASE" ]; then
					COMMANDS+=("bash -i -c \"$startup_script_location -x move_base -p 1132$ROBO $KEEP $@\"")
				fi
	if [ -n "$ROS_LAUNCH_ROBOT" ]; then
	    COMMANDS+=("bash -i -c \"$startup_script_location -x roslaunch $ROS_LAUNCH_ROBOT -p $ROS_MASTER_PORT $KEEP $@\"")
	fi
    	done
    fi
    #start refbox
    COMMANDS+=("bash -i -c \"$startup_script_location -x $REFBOX  $KEEP $@ -- $REFBOX_ARGS\"")
    #start refbox frontend
    COMMANDS+=("bash -i -c \"$startup_script_location -x refbox-frontend $KEEP $@\"")

    # start mongodb central instance
    if $START_MONGODB ; then
        MONGODB_DBPATH=$(mktemp -d --tmpdir mongodb-27017-XXXXXXXXXXXX)
        COMMANDS+=("bash -i -c \"mongod --port 27017 --dbpath $MONGODB_DBPATH | tee mongodb.log \"")
    fi
    #start fawkes for robotinos
    for ((ROBO=$FIRST_ROBOTINO_NUMBER ; ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;ROBO++))
    do
	COMMANDS+=("bash -i -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 5; $startup_script_location -x fawkes -p 1132$ROBO -i robotino$ROBO $KEEP $CONF $ROS $ROS_LAUNCH_MAIN $ROS_LAUNCH_ROBOT $GDB $META_PLUGIN $DEBUG $DETAILED -f $FAWKES_BIN $SKIP_EXPLORATION $@\"")
	FAWKES_USED=true
    done

    if $START_CENTRAL_AGENT ; then
        if [ $NUM_CYAN -gt 0 ] ; then
		    ROBO=11
		    COMMANDS+=("bash -i -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 5; $startup_script_location -x fawkes -i robotino$ROBO $KEEP $CONF $GDB -m $CENTRAL_AGENT $DEBUG $DETAILED -f $FAWKES_BIN $SKIP_EXPLORATION $@\"")
        fi
        if [ $NUM_MAGENTA -gt 0 ] ; then
		    ROBO=12
		    COMMANDS+=("bash -i -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 5; $startup_script_location -x fawkes -i robotino$ROBO $KEEP $CONF $GDB -m $CENTRAL_AGENT $DEBUG $DETAILED -f $FAWKES_BIN $SKIP_EXPLORATION $@\"")
        fi
        echo "fawkes/bbsync/peers:" > $FAWKES_DIR/cfg/robotino_${ROBO}_generated.yaml
        for ((CURR_ROBO=$FIRST_ROBOTINO_NUMBER ; CURR_ROBO<$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)) ;CURR_ROBO++))
        do
            echo "  robot$CURR_ROBO/active: true" >> $FAWKES_DIR/cfg/robotino_${ROBO}_generated.yaml
        done
        for ((CURR_ROBO=$(($FIRST_ROBOTINO_NUMBER+$NUM_ROBOTINOS)); CURR_ROBO<4 ;CURR_ROBO++))
        do
            echo "  robot$CURR_ROBO/active: false" >> $FAWKES_DIR/cfg/robotino_${ROBO}_generated.yaml
        done
    fi

    if $START_ASP_PLANER
    then
	COMMANDS+=("bash -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 5; $startup_script_location -x asp -p ${ROS_MASTER_URI##*:} $KEEP $CONF $ROS $ROS_LAUNCH_MAIN $ROS_LAUNCH_ROBOT $GDB $DEBUG $DETAILED -f $FAWKES_BIN $SKIP_EXPLORATION $@\"")
    fi

    #start fawkes for communication, llsfrbcomm and eventually statistics
    if $START_GAZEBO ; then
        comm_plugin=comm
    else
        comm_plugin=comm-no-gazebo
    fi
	COMMANDS+=("bash -i -c \"export TAB_START_TIME=$(date +%s); $script_path/wait-at-first-start.bash 0; $startup_script_location -x $comm_plugin $DEBUG $KEEP $SHUTDOWN $@\"")

    PREFIXED_COMMANDS=("${COMMANDS[@]/#/${SUBTERM_PREFIX}}")
    SUFFIXED_COMMANDS=("${PREFIXED_COMMANDS[@]/%/${SUBTERM_SUFFIX}}")
    echo "Executing $TERM_COMMAND ${SUFFIXED_COMMANDS[@]} ${TERM_COMMAND_END}"
    eval "$TERM_COMMAND ${SUFFIXED_COMMANDS[@]} ${TERM_COMMAND_END}"

    if $FAWKES_USED && $START_GAZEBO
    then
	# publish initial poses
	echo "publish initial poses"
	$initial_pose_script_location $CONF -C $NUM_CYAN -M $NUM_MAGENTA -d
    else
	echo "Skipped publishing poses"
    fi

		if [ -n "$START_GAME" ]; then
				if [ "$(command -v rcll-refbox-instruct)" == "" ]; then
						echo "rcll-refbox-instruct not found, not built or old version?"
				else
						rcll-refbox-instruct -w
						echo "Starting game (Phase: $START_GAME ${TEAM_CYAN:+Cyan: ${TEAM_CYAN}}${TEAM_MAGENTA:+ Magenta: ${TEAM_MAGENTA}})"
						rcll-refbox-instruct -p SETUP -s RUNNING ${TEAM_CYAN:+-c ${TEAM_CYAN}}${TEAM_MAGENTA:+-m ${TEAM_MAGENTA}}
						rcll-refbox-instruct -n $NUM_ROBOTINOS -W60 || (stop_simulation; exit 1)
						rcll-refbox-instruct -p $START_GAME -s RUNNING
				fi
		fi

    else
    usage
fi

# vim:et:sw=4:ts=4
