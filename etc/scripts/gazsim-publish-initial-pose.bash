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
                       This is retrieved from the config, e.g., /initial-pose/C-R1/x
   -c CONF             Configuration base dir
   -C N                Assume N available robots of team cyan (default: 3)
   -M N                Assume N available robots of team magenta (default: 0)
EOF
}

wait_for_amcl()
{
  if [ $# -lt 1 ] ; then
    echo "wait_for_amcl: Missing argument!"
    exit 1
  fi
  echo -n "Waiting for AMCL to be ready on $1 ."
  while [ "$(${FAWKES_DIR}/bin/ffplugin -r $1 | grep -w amcl)" = "" ]
  do
    echo -n "."
    sleep 1
  done
  echo " sending position!"
}

#check options

PORT=1910
X=
Y=
O0=0.0
O1=0.0
O2=0.0
O3=1.0
CN=3
MN=0
CONF=
DEFAULT_POS=
while getopts “hx:y:p:o:dc:C:M:” OPTION
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
	 c)
	     CONF=gazsim-configurations/$OPTARG
	     ;;
	 C)
	     CN=$OPTARG
	     ;;
	 M)
	     MN=$OPTARG
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
		 DEFAULT_POS=true
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

if [ -n "$DEFAULT_POS" ]; then
	for i in $(seq 1 $CN) $(seq 4 $((MN + 3))); do
		if [ -z "$CONF" ]; then
			config_file=config.yaml
		else
			config_file=$CONF/robotino$i.yaml
		fi
		r=C-R$i
		if (( i > 3 )); then
			r=M-R$(( i - 3 ))
		fi
		h=$($FAWKES_DIR/bin/ffconfig -f $config_file -q get /initial-pose/$r/host)
		x=$($FAWKES_DIR/bin/ffconfig -f $config_file -q get /initial-pose/$r/x)
		y=$($FAWKES_DIR/bin/ffconfig -f $config_file -q get /initial-pose/$r/y)
		z=$($FAWKES_DIR/bin/ffconfig -f $config_file -q get /initial-pose/$r/z)
		mapfile -t ori < <($FAWKES_DIR/bin/ffconfig -f $config_file -q get /initial-pose/$r/ori)
		if [[ -z "$h" || -z "$x" || -z "$y" || -z "$z" || -z "$ori" ]]; then
			echo "Some value not found for $p-R$i"
			exit 2
		fi
		wait_for_amcl $h
		$FAWKES_DIR/bin/ffset_pose -r $h -t 2.0 --  $x $y $z $ori
	done
else
	if [[ -z $X ]] || [[ -z $Y ]]
	then
		usage
		exit 1
	fi
	$FAWKES_DIR/bin/ffset_pose -r localhost:${PORT} $X $Y 0.0 $O0 $O1 $O2 $O3
fi
