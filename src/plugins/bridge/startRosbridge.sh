

OPEN_COMMAND="gnome-terminal"


#Launch the Rosbridge and Tf2_web_republisher instace for each ros core 
OPEN_COMMAND="$OPEN_COMMAND --tab -t R1_rosbridge -e 'bash -c \"export ROS_MASTER_URI=http://localhost:11311; rosparam set /rosbridge/port 9090; rosrun rosbridge_server rosbridge.py; exec bash\"'"

OPEN_COMMAND="$OPEN_COMMAND --tab -t R1_tf2 -e 'bash -c \"export ROS_MASTER_URI=http://localhost:11311; rosrun tf2_web_republisher tf2_web_republisher; exec bash\"'"
 

eval $OPEN_COMMAND

