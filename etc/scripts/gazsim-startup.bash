#!/bin/bash
# Automated startup of a Gazebo simulation

usage()
{
cat << EOF
usage: $0 options

This script starts a specified program for simulation

OPTIONS:
   -h             Show this message
   -x gazebo|fawkes|comm|roscore|move_base|refbox|refbox_shell  
                  Start specified program
   -c arg         (when using fawkes) 
                  Use a specific configuration-folder
                  in cfg/gazsim-configurations/
   -p arg         Specify ros port
   -l             (when using gazebo)
                  Run Gazebo headless
   -r             (when using fawkes)
                  Start ros
   -s             (when using fawkes)
                  Keep statistics and shutdown after game
   -i robotino[1|2|3]
                   Robotino instance
EOF
}
 
#check options

COMMAND=
CONF=gazsim-configurations/default
VISUALIZATION=visualization
ROS=false
SHUTDOWN=
PORT=11311
ROBOTINO=
while getopts “hx:c:lrsp:i:” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         c)
	     CONF=gazsim-configurations/$OPTARG
             ;;
         l)
	     VISUALIZATION=headless
             ;;
         x)
	     COMMAND=$OPTARG
             ;;
	 r)
	     ROS=true
	     ;;
	 s)
	     SHUTDOWN=,gazsim-llsf-statistics,gazsim-llsf-control
	     ;;
	 p)
	     PORT=$OPTARG
	     ;;
	 i)
	     ROBOTINO=$OPTARG
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
    gazebo ) 
	#use optirun if available
	opti=$(command -v optirun)
	if [ $VISUALIZATION == headless ]
	then
	    $opti gzserver ~/.gazebo/plugins/llsf/llsf.world
	else
	    $opti gazebo ~/.gazebo/plugins/llsf/llsf.world
	fi
	;;
    fawkes ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	if [ $ROS == true ] ; then
	    robotino_plugins=gazsim-full-robotino-ros,clips,clips-agent,clips-protobuf,clips-motor-switch,gazsim-vis-localization
	else
	    robotino_plugins=gazsim-full-robotino
	fi
	~/fawkes-robotino/bin/fawkes -c $CONF/$ROBOTINO.yaml -p $robotino_plugins
	;;
    comm )
	comm_plugins=gazsim-organization$SHUTDOWN
	~/fawkes-robotino/bin/fawkes -p $comm_plugins
	;;
    roscore ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	roscore -p $PORT
	;;
    move_base ) 
	export ROS_MASTER_URI=http://localhost:$PORT
	#rosparam set /use_sim_time true
	roslaunch ~/fawkes-robotino/cfg/move_base_robotino/launch/move_base.launch
	;;
    refbox )
	~/llsf-refbox/bin/llsf-refbox
	;;
    refbox-shell )
	~/llsf-refbox/bin/llsf-refbox-shell
	;;
esac
