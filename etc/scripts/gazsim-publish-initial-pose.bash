#!/bin/bash
# Publishes the initial Robotino Pose with rostopic

usage()
{
cat << EOF
usage: $0 options

This script publishes the initial Robotino Pose with rostopic

OPTIONS:
   -h             Show this message
   -x arg         Initial x coordinate
   -y arg         Initial y coordinate
   -p arg         Ros port 
EOF
}
 
#check options

PORT=11311
X=
Y=
while getopts “hx:y:p:” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
	 x)
	     X=$OPTARG
	     ;;
	 y)
	     Y=$OPTARG
	     ;;
	 p)
	     PORT=$OPTARG
	     ;;
         ?)
             usage
             exit
             ;;
     esac
done

if [[ -z $X ]] || [[ -z $Y ]]
then
     usage
     exit 1
fi

# Result of rostopic echo initialpose :
# header: 
#   seq: 0
#   stamp: 
#     secs: 0
#     nsecs: 0
#   frame_id: /map
# pose: 
#   pose: 
#     position: 
#       x: 0.463881731033
#       y: 0.602762699127
#       z: 0.0
#     orientation: 
#       x: 0.0
#       y: 0.0
#       z: -0.124706299333
#       w: 0.992193700296
#   covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
# ---

#choose ros master uri
export ROS_MASTER_URI=http://localhost:$PORT
#Publish msg
rostopic pub -1 initialpose geometry_msgs/PoseWithCovarianceStamped  "{header:  {seq: 1, stamp: {secs: 0, nsecs: 0}, frame_id: /map}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}"
