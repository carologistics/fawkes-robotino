#!/bin/bash
# This script lauches a RosBridge server on a given port (or 9090 by default) and the tf2_web_rebuplisher necessary to throttle the TFs send from ROS to RosBridge
# the script will server will use the ros node statred on the currently set ROS_MASTER_URI


usage()
{
cat << EOF
usage: $0 options

This scrpit launches a Rosbridge server on a certian port and and a tf2_wed_republisher node 

OPTIONS:
   -h             Show this message
   -p arg         port to start Rosbridge on
   -u arg         Set the ROS_MASTER_URI
EOF
}

# check options
ROSBRIDGE_PORT=9090
MY_ROS_MASTER=$ROS_MASTER_URI



while getopts “hp:u:” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         p)
       ROSBRIDGE_PORT=$OPTARG
             ;;

        u)
        if [ ${#OPTARG} -gt 4 ]
        then
            MY_ROS_MASTER="http://localhost:$OPTARG"
        else
            MY_ROS_MASTER=$OPTARG
        fi 
             ;;
         
         ?)
             usage
             exit
             ;;
     esac
done


echo $MY_ROS_MASTER
echo $ROSBRIDGE_PORT


OPEN_COMMAND="gnome-terminal"


# #Launch the Rosbridge and Tf2_web_republisher instace for each ros core 

if [ $ROSBRIDGE_PORT == "9090" ]
then 
    OPEN_COMMAND="$OPEN_COMMAND --tab -t R1_rosbridge -e 'bash -c \"export ROS_MASTER_URI=$MY_ROS_MASTER; roslaunch rosbridge_server rosbridge_websocket.launch; exec bash\"'"
fi

if [ $ROSBRIDGE_PORT == "8080" ]
then 
    OPEN_COMMAND="$OPEN_COMMAND --tab -t R2_rosbridge -e 'bash -c \"export ROS_MASTER_URI=$MY_ROS_MASTER; roslaunch rosbridge_server rosbridge_websocket_8080.launch; exec bash\"'"
fi

if [ $ROSBRIDGE_PORT == "7070" ]
then 
    OPEN_COMMAND="$OPEN_COMMAND --tab -t R3_rosbridge -e 'bash -c \"export ROS_MASTER_URI=$MY_ROS_MASTER; roslaunch rosbridge_server rosbridge_websocket_7070.launch; exec bash\"'"
fi


OPEN_COMMAND="$OPEN_COMMAND --tab -t tf2 -e 'bash -c \"export ROS_MASTER_URI=$MY_ROS_MASTER; rosrun tf2_web_republisher tf2_web_republisher; exec bash\"'"
OPEN_COMMAND="$OPEN_COMMAND --tab -t marker -e 'bash -c \"export ROS_MASTER_URI=$MY_ROS_MASTER; rosrun visualization_marker_tutorials arrow; exec bash\"'"
 

eval $OPEN_COMMAND


#TODO:
#check the ros running distro and adapt commands for feurte (like rosrun and the bridgeport)
#let the port be passed to the ROSBRIDGE launch script
#Try to idnetify which robot we will ge the visualization_marker topic from and either start it 
