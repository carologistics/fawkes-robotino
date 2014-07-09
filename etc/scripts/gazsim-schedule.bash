#!/bin/bash
# Automated startup of multiple gazsim-simulations in a shcedule with different configurations

usage()
{
cat << EOF
usage: $0 options
]
This script automates the execution of multiple Gazebo-simulations with different configurations

OPTIONS:
   -h      Show this message
   -c arg  A configuration-folder in cfg/gazsim-configurations/ to test 
           (you can define multiple by "-c c1 -c c2 ...") 
   -l      Run Gazebo headless
   -n arg  The amount of test-runs
   -d      Run a detailed simulation (e.g. with simulated vision)
EOF
}

replace_config() #args: 1:config-name 2:new value
{
    echo "sed -i s/$1:.*/$1: $2/"
    sed -i "s/$1:.*/$1: $2/" $FAWKES_DIR/cfg/conf.d/gazsim.yaml
}

restore_record() #args 1: file
{
    END_FLAG=$(grep '</gazebo_log>' $1)
    if [[ -z $END_FLAG ]]
    then
	echo '</gazebo_log>' >> $1
    fi
}
 
#check options

NUM_CONF=0
HEADLESS=
DETAILED=
NUM_RUNS=1
while getopts “hc:ln:d” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         c)
	     CONFIGURATIONS[$NUM_CONF]=$OPTARG
	     let "NUM_CONF++"

	     echo Configuration $NUM_CONF is $OPTARG
	     ;;
         l)
	     HEADLESS=-l
             ;;
         n)
	     NUM_RUNS=$OPTARG
             ;;
         d)
	     DETAILED=-d
             ;;
         ?)
             usage
             exit
             ;;
     esac
done

if [[ -z $CONFIGURATIONS ]]
then
     echo Please specify at least one configuration
     exit 1
fi

#run simulations

STARTUP_SCRIPT_LOCATION=$FAWKES_DIR/bin/gazsim.bash

TIME=$(date +'%y_%m_%d_%H_%M')


for ((RUN=1 ; RUN<=$NUM_RUNS ;RUN++))
do
    for CONF in ${CONFIGURATIONS[@]}
    do
	#create and go to log folder
	cd $FAWKES_DIR
	export FAWKES_DIR_FOR_SED=$(echo $FAWKES_DIR | sed "s/\//\\\\\//g")
	mkdir -p "gazsim-logs/$TIME/${CONF}_$RUN"
	cd "gazsim-logs/$TIME/${CONF}_$RUN"

	echo Executing simulation-run $RUN with configuration $CONF

	#set config values
	replace_config run $RUN
	replace_config configuration-name "\"$CONF\""
	replace_config collection "\"test_$TIME\""
	replace_config log "\"$FAWKES_DIR_FOR_SED\/gazsim-logs\/$TIME\/$CONF\_$RUN\"" #creepy string because of sed
        

	#start simulation
	REPLAY_PATH="$FAWKES_DIR/gazsim-logs/$TIME/${CONF}_$RUN"
	$STARTUP_SCRIPT_LOCATION -x start -r -a -s $HEADLESS -c $CONF -e $REPLAY_PATH $DETAILED
        
	#wait for shutdown of simulation (caused by gazsim-llsf-statistics if the game is over)
	echo Waiting for shutdown of the simulation
	for (( ; ; ))
	do
	    sleep 30s
	    #check if simulation is still running
	    GAZEBO=$(ps -a | grep -i 'gzserver\|fawkes\|roscore\|llsf-refbox' | wc -l)
            if [ $GAZEBO -eq 0 ]
	    then
		echo Simulation-run $RUN with configuration $CONF finished
		break
	    fi

	    #check if something went wrong (workaround for unsolved crashs)
	    GZSERVER=$(ps -a | grep -i 'gzserver' | wc -l)
	    FAWKES=$(ps -a | grep -i 'fawkes' | wc -l)
	    REFBOX=$(ps -a | grep -i 'llsf-refbox' | wc -l)
	    ROS=$(ps -a | grep -i 'roscore' | wc -l)
	    if [ $GZSERVER -lt 1 ] || [ $FAWKES -lt 4 ] || [ $REFBOX -lt 1 ] || [ $ROS -lt 1 ]
	    then
		echo something went wrong
		echo restarting run
		$STARTUP_SCRIPT_LOCATION -x kill
		$STARTUP_SCRIPT_LOCATION -x start -r -a -s $HEADLESS -c $CONF -e $REPLAY_PATH
		sleep 30
	    fi
	    
	done
	#wait until the record is stored
	sleep 10s
	restore_record "$REPLAY_PATH/state.log"
    done
done
