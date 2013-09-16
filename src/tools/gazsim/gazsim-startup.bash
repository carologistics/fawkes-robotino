#!/bin/bash
# Automated startup of a Gazebo simulation
# programm to start indicated by argument $1
# ros_port  indicated by argument $2
# $3 fakes purpose (robotino1, robotino2, robotino3 or comm)
# $4 ros?

echo "$1"
echo "$2"
echo "$3"

case $1 in
    gazebo ) 
	gazebo ~/.gazebo/plugins/llsf/llsf.world
	;;
    fawkes ) 
	export ROS_MASTER_URI=http://localhost:$2
	if [[ $4 = ros ]] ; then
	    robotino_plugins=gazsim-full-robotino-ros,clips,clips-agent,clips-protobuf,clips-motor-switch
	else
	    robotino_plugins=gazsim-full-robotino
	fi
	
	
	comm_plugins=gazebo,gazsim-llsfrbcomm,gazsim-comm
	case $3 in
	    robotino1 ) ~/fawkes-robotino/bin/fawkes -c robotino1.yaml -p $robotino_plugins
		;;
	    robotino2 ) ~/fawkes-robotino/bin/fawkes -c robotino2.yaml -p $robotino_plugins
		;;
	    robotino3 ) ~/fawkes-robotino/bin/fawkes -c robotino3.yaml -p $robotino_plugins
		;;
	    comm ) ~/fawkes-robotino/bin/fawkes -p $comm_plugins
		;;
	esac
	;;
    roscore ) 
	export ROS_MASTER_URI=http://localhost:$2
	roscore -p $2
	;;
    move_base ) 
	export ROS_MASTER_URI=http://localhost:$2
	roslaunch ~/fawkes-robotino/cfg/move_base_robotino/launch/move_base.launch
	;;
    refbox )
	~/llsf-refbox/bin/llsf-refbox
	;;
    refbox_shell )
	~/llsf-refbox/bin/llsf-refbox-shell
	;;
esac
