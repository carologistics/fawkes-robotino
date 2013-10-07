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
EOF
}

replace_config() #args: 1:config-name 2:new value
{
    sed -i "s/$1:.*/$1: $2/" ~/fawkes-robotino/cfg/conf.d/gazsim.yaml
}
 
#check options

NUM_CONF=0
HEADLESS=
NUM_RUNS=1
while getopts “hc:ln:” OPTION
do
     case $OPTION in
         h)
             usage
             exit 1
             ;;
         c)
	     CONFIGURATIONS[0]=$OPTARG
	     let "NUM_CONF++"

	     echo Configuration $NUM_CONF is $OPTARG
	     ;;
         l)
	     HEADLESS=-l
             ;;
         n)
	     NUM_RUNS=$OPTARG
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

STARTUP_SCRIPT_LOCATION=~/fawkes-robotino/bin/gazsim.bash

TIME=$(date +'%y_%m_%d_%H_%M')

for ((RUN=1 ; RUN<=$NUM_RUNS ;RUN++))
do
    for CONF in ${CONFIGURATIONS[@]}
    do
	echo Executing simulation-run $RUN with configuration $CONF
	replace_config run $RUN
	replace_config configuration-name "\"$CONF\""
	replace_config collection "\"test_$TIME\""
	replace_config replay "\"~\/.gazebo\/log\/gazsim-runs\/$TIME\/$CONF\_$RUN\"" #creepy string because of sed
        REPLAY_PATH=~/.gazebo/log/gazsim-runs/$TIME/$CONF\_$RUN
	$STARTUP_SCRIPT_LOCATION -x start -r -s $HEADLESS -c $CONF -e $REPLAY_PATH
        #wait for shutdown of simulation (caused by gazsim-llsf-statistics if the game is over)
	echo Waiting for shutdown of the simulation
	for (( ; ; ))
	do
	    sleep 10s
	    #check if gazebo is still running
	    GAZEBO=$(ps -a | grep -i gzserver | wc -l)
            if [ $GAZEBO -eq 0 ]
	    then
		echo Simulation-run $RUN with configuration $CONF finished
		break
	    fi
	done
    done
done
