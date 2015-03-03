#!/bin/bash
# Publishes the initial Robotino Pose with rostopic

usage()
{
cat << EOF
usage: $0 options

This script publishes the initial Robotino Pose with rostopic

OPTIONS:
   -h                  Show this message
   -x arg              Initial x coordinate
   -y arg              Initial y coordinate
   -p arg              Fawkes remote as port 
   -o arg arg arg arg  Initial orientation (quaternion)
   -d                  Publish default initial localization for all three robots
EOF
}
 
#check options

PORT=1910
X=
Y=
O0=0.0
O1=0.0
O2=0.0
O3=1.0
while getopts “hx:y:p:o:d” OPTION
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
	 o)
	     # option with 4 option arguments
             if [ $# -lt $((OPTIND + 2)) ]
             then    
		 echo "$IAM: orientation quaternion needs four input arguments"
                 usage
                 exit 1
             fi
             OPTINDplus1=$((OPTIND + 1))
             OPTINDplus2=$((OPTIND + 2))
             O0=$OPTARG
	     eval O1=\$$OPTIND
             eval O2=\$$OPTINDplus1
             eval O3=\$$OPTINDplus2
             sc=2
             ;;
	 p)
	     PORT=$OPTARG
	     ;;
	 d)
	     script_path=$FAWKES_DIR/bin
	     set_pose=$script_path/ffset_pose
	     $set_pose -r localhost:1921 -t 2.0 --  3.5  -0.5 0.0  0.0 0.0 0.7 0.7
	     $set_pose -r localhost:1922 -t 2.0 --  4.5 -0.5 0.0  0.0 0.0 0.7 0.7
	     $set_pose -r localhost:1923 -t 2.0 --  5.5  -0.5 0.0  0.0 0.0 0.7 0.7
	     $set_pose -r localhost:1924 -t 2.0 -- -3.5  -0.5 0.0  0.0 0.0 0.7 0.7
	     $set_pose -r localhost:1925 -t 2.0 -- -4.5 -0.5 0.0  0.0 0.0 0.7 0.7
	     $set_pose -r localhost:1926 -t 2.0 -- -5.5  -0.5 0.0  0.0 0.0 0.7 0.7
	     exit 1
	     ;;
         ?)
             usage
             exit
             ;;
     esac
     if [ $OPTIND != 1 ] # This test fails only if multiple options are stacked
                         # after a single -.
     then    shift $((OPTIND - 1 + sc))
         OPTIND=1
     fi
done

if [[ -z $X ]] || [[ -z $Y ]]
then
     usage
     exit 1
fi

$FAWKES_DIR/bin/ffset_pose $X $Y 0.0 $O0 $O1 $O2 $O3

