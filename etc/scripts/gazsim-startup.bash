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
   -s             Keep statistics and shutdown after game
   -i robotino[1|2|3]
                  Robotino instance
   -d             Detailed simulation (e.g. simulated webcam)
   -a             Start with agent

  GAZEBO:
   -e arg         Record Replay
EOF
}
 
#check options

#default values
COMMAND=
CONF=gazsim-configurations/default
ROS=
SHUTDOWN=
PORT=11311
ROBOTINO=
REPLAY=
VISION=,gazsim-light-front,gazsim-puck-detection
AGENT=
FAWKES_BIN=$FAWKES_DIR/bin
while getopts “hx:c:lrsp:i:e:da4” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         c)
	     CONF=gazsim-configurations/$OPTARG
             ;;
         x)
	     COMMAND=$OPTARG
             ;;
	 r)
	     ROS=,gazsim-meta-ros
	     ;;
	 s)
	     SHUTDOWN=,mongodb,gazsim-llsf-statistics,gazsim-llsf-control
	     ;;
	 p)
	     PORT=$OPTARG
	     ;;
	 i)
	     ROBOTINO=$OPTARG
	     ;;
	 e)
	     REPLAY=-r\ --record_path\ $OPTARG
	     ;;
	 d)
	     VISION=,gazsim-meta-vision
	     ;;
	 a)
	     AGENT=,clips,clips-agent,clips-protobuf,clips-motor-switch,clips-webview
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


#ulimit -c unlimited

case $COMMAND in
    gzserver ) 
	# change Language (in german there is an error that gazebo can not use a number with comma)
	export LANG="en_US"
	gzserver $REPLAY $GAZEBO_MODEL_PATH/llsf_world_2014/llsf.world
	;;
    gzclient ) 
	# change Language (in german there is an error that gazebo can not use a number with comma)
	export LANG="en_US"
	#use optirun if available
	opti=$(command -v optirun)
	$opti gzclient
	;;
    fawkes ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	robotino_plugins=gazsim-meta-robotino$ROS$VISION$AGENT
	$FAWKES_BIN/fawkes -c $CONF/$ROBOTINO.yaml -p $robotino_plugins
	;;
    comm )
	comm_plugins=gazsim-organization$SHUTDOWN
	$FAWKES_BIN/fawkes -p $comm_plugins
	;;
    roscore ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	roscore -p $PORT
	;;
    move_base ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	#rosparam set /use_sim_time true
	roslaunch $FAWKES_DIR/cfg/move_base_robotino/launch/move_base.launch
	;;
    refbox )
	$LLSF_REFBOX_DIR/bin/llsf-refbox
	;;
    refbox-shell )
	$LLSF_REFBOX_DIR/bin/llsf-refbox-shell
	;;
esac
