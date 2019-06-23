#!/bin/bash
# Automated startup of a Gazebo simulation

usage()
{
cat << EOF
usage: $0 options

This script starts a specified program for simulation

OPTIONS:
  COMMON:
   -h             Show this message
   -x gzserver|gzclient|fawkes|comm|roscore|move_base|refbox|refbox_shell  
                  Start specified program
   -p arg         Specify ros port

  FAWKES:
   -c arg         Use a specific configuration-folder
                  in cfg/gazsim-configurations/
   -r             Start with ros
   --ros-launch-main  Run launch file for main (non-robot) roscore
   --ros-launch   Run launch file for robot roscore
   -s             Keep statistics and shutdown after game
   -i robotino[1|2|3]
                  Robotino instance
   -d             Detailed simulation (e.g. simulated webcam)
   -a             Start with agent
   -f arg         Path to the fawkes bin folder to use
                  ($FAWKES_DIR/bin by default)
   -g             Run Fawkes in gdb
   -v             Run Fawkes in valgrind
   -t             Skip Exploration and add all navgraph points
   -w             Alternative world file
  GAZEBO:
   -e arg         Record Replay
EOF
}
 
#check options

#default values
COMMAND=
CONF=gazsim-configurations/default
ROS=-no-ros
ROS_LAUNCH=
SHUTDOWN=
PORT=11311
ROBOTINO=
REPLAY=
VISION=,gazsim-meta-robotino-vision-high-level
AGENT=
FAWKES_BIN=$FAWKES_DIR/bin
KEEP=
GDB=
SKIP_EXPLORATION=
GAZEBO_WORLD=$GAZEBO_WORLD_PATH

OPTS=$(getopt -o "hx:c:lrksn:e:dm:aof:p:gvi:tw:" -l "ros,ros-launch:" -- "$@")
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
             usage
             exit 1
             ;;
         -c)
	     CONF=gazsim-configurations/$OPTARG
             ;;
         -x)
	     COMMAND=$OPTARG
             ;;
	 -r|--ros)
	     ROS=-ros
	     ;;
	 --ros-launch)
	     ROS_LAUNCH="$OPTARG"
	     ;;
	 -g)
	     if [ -n "$GDB" ]; then
				echo "Can pass only either valgrind or GDB, not both"
        exit
       fi
	     GDB="gdb -ex run --args"
	     ;;
	 -v)
	     if [ -n "$GDB" ]; then
				echo "Can pass only either valgrind or GDB, not both"
        exit
       fi
	     GDB="valgrind --track-origins=yes"
	     ;;
	 -s)
	     SHUTDOWN=,mongodb,gazsim-llsf-statistics,gazsim-llsf-control
	     ;;
	 -p)
	     PORT=$OPTARG
	     ;;
	 -w)
	     GAZEBO_WORLD=$OPTARG
	     ;;
	 -i)
	     ROBOTINO=$OPTARG
	     ;;
	 -e)
	     REPLAY=-r\ --record_path\ $OPTARG
	     ;;
	 -d)
	     VISION=,gazsim-meta-robotino-vision-low-level
	     ;;
	 -a)
	     AGENT=,gazsim-meta-agent
	     ;;
         -k)
             KEEP=yes
             ;;
         -m)
             META_PLUGIN=$OPTARG
             ;;
	 -t)
	     SKIP_EXPLORATION=",gazsim-navgraph-generator"
	     ;;
	 -f)
	     FAWKES_BIN=$OPTARG
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

if [ -n $LLSF_REFBOX_DIR ] ; then
    export PATH=$LLSF_REFBOX_DIR/bin:$PATH
fi

if [[ -z $GAZEBO_WORLD_PATH ]]
then
     echo "Error: \$GAZEBO_WORLD_PATH is not set. Please set it in your .bashrc"
     exit 1
fi


#ulimit -c unlimited

case $COMMAND in
    gazebo )
	# change Language (in german there is an error that gazebo can not use a number with comma)
	export LC_ALL="C"
	( gzserver $REPLAY $GAZEBO_WORLD_PATH & ); sleep 10s; gzclient
	;;
    gzserver ) 
	# change Language (in german there is an error that gazebo can not use a number with comma)
	export LC_ALL="C"
	gzserver $REPLAY $GAZEBO_WORLD $@
	;;
    gzclient ) 
	# change Language (in german there is an error that gazebo can not use a number with comma)
	export LC_ALL="C"
	#use optirun if available
	#opti=$(command -v optirun)
	$opti gzclient $@
	;;
    fawkes )
	ulimit -c unlimited
	export ROS_MASTER_URI=http://localhost:$PORT
	if [ -n "$META_PLUGIN" ] ; then
		robotino_plugins=$META_PLUGIN
	else
	  robotino_plugins=gazsim-meta-robotino$ROS$VISION$AGENT$SKIP_EXPLORATION
	fi
	$GDB $FAWKES_BIN/fawkes -c $CONF/$ROBOTINO.yaml -p $robotino_plugins $@
	if [ -n "$GDB" ]; then
		echo Fawkes exited, press return to close shell
		read
	fi
	;;
    comm )
	comm_plugins=gazsim-organization$SHUTDOWN
	$FAWKES_BIN/fawkes -p $comm_plugins $@
	;;
    comm-no-gazebo )
	comm_plugins=gazsim-comm
	$FAWKES_BIN/fawkes -p $comm_plugins $@
	;;
    asp )
	ulimit -c unlimited
	export ROS_MASTER_URI=http://localhost:$PORT
	robotino_plugins=asp-planner-sim-2016$SKIP_EXPLORATION
	$GDB $FAWKES_BIN/fawkes -c $CONF/asp-planner.yaml -p $robotino_plugins
	if [ -n "$GDB" ]; then
		echo Fawkes exited, press return to close shell
		read
	fi
	;;
    roscore ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	roscore -p $PORT $@
	;;
    roslaunch)
	export ROS_MASTER_URI=http://localhost:$PORT
	roslaunch $@ --wait ${ROS_LAUNCH%:*} ${ROS_LAUNCH##*:} 
	;;
    move_base ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	rosparam set /use_sim_time true
	export ROS_PACKAGE_PATH=$FAWKES_DIR/cfg/move_base_robotino:$ROS_PACKAGE_PATH
	roslaunch $@ --wait robotino_move_base robotino_move_base_simu.launch
	;;
    refbox )
	llsf-refbox $@
	;;
    refbox-shell )
        # wait some time such that the terminal has the final size
	sleep 3
	llsf-refbox-shell $@
	;;
esac


if [[ "$KEEP" == "yes" ]]; then
	echo -n "Press ENTER to exit..."
	read
fi

