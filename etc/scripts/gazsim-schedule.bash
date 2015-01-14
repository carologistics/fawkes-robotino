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

#function to add a team from the configuration file
addTeam() #args 1:teamname 2:fawkes-robotino-branch 3:fawkes-branch 4:configuration 5:number-robots
{
    TEAMS[$NUM_TEAMS]=$1
    FAWKES_ROBOTINO_BRANCHES[$NUM_TEAMS]=$2
    FAWKES_BRANCHES[$NUM_TEAMS]=$3
    CONFIGURATIONS[$NUM_TEAMS]=$4
    NUMBER_ROBOTS[$NUM_TEAMS]=$5

    echo Adding Team ${TEAMS[$NUM_TEAMS]} with branches ${FAWKES_ROBOTINO_BRANCHES[$NUM_TEAMS]} and ${FAWKES_BRANCHES[$NUM_TEAMS]}, configuration ${CONFIGURATIONS[$NUM_TEAMS]} and ${NUMBER_ROBOTS[$NUM_TEAMS]} robots.

    let "NUM_TEAMS++"
}

#check options
NUM_TEAMS=0
PREVIOUS_BRANCH=
while getopts “hc:b:ln:d” OPTION
do
    case $OPTION in
        h)
            usage
            exit 1
            ;;
        ?)
            usage
            exit
            ;;
    esac
done

#read out configuration file
source $FAWKES_DIR/cfg/gazsim-configurations/automated-competition-conf.bash

if $HEADLESS
then
    HEADLESS=-l
else
    HEADLESS=
fi

if [ $NUM_TEAMS -lt 2 ]
then
    echo Please specify at least two teams
    exit 1
fi

#setup log folder
TIME=$(date +'%y_%m_%d_%H_%M')
START_PATH=$COMPETITION_LOG_PATH/$COMPETITION_NAME$TIME
echo $START_PATH
mkdir -p "$START_PATH"

# #checkout and compile team code
# mkdir -p "$COMPETITION_LOG_PATH/teams"
# for ((TEAM=0 ; TEAM<$NUM_TEAMS ;TEAM++))
# do
#     cd "$COMPETITION_LOG_PATH/teams"
#     echo Preparing the code of team ${TEAMS[$TEAM]}
#     # Does the code of the ream already exist?
#     if [ -d "${TEAMS[$TEAM]}" ]; then
# 	echo Directory with team code already exists, updating it
#     else
# 	echo Directory with team code missing, cloning it
# 	git clone --recursive git@git.fawkesrobotics.org:fawkes-robotino.git ${TEAMS[$TEAM]}
#     fi
#     cd "${TEAMS[$TEAM]}"
#     git fetch
#     git reset --hard HEAD
#     git checkout -b $COMPETITION_NAME$TIME ${FAWKES_ROBOTINO_BRANCHES[$TEAM]}
#     cd fawkes
#     git fetch
#     git reset --hard HEAD
#     git checkout -b $COMPETITION_NAME$TIME ${FAWKES_BRANCHES[$TEAM]}
#     cd ..

#     #Compile code
#     echo Compiling...
#     COMPILE_OUTPUT="$(make all -j8)"
#     if [ "$COMPILE_OUTPUT" == *"Error"* ]
#     then
# 	echo "${TEAMS[$TEAM]}" has a compile error
# 	echo You can find the compile errors here:
# 	pwd
# 	touch compile_errors.txt
# 	echo "$COMPILE_OUTPUT" > compile_errors.txt
# 	exit 1
#     else
# 	echo Compiling successful
# 	#####DEBUG
# 	touch compile_errors.txt
# 	echo "$COMPILE_OUTPUT" > compile_errors.txt
#     fi

# done

#run simulations
cd "$START_PATH"
STARTUP_SCRIPT_LOCATION=$FAWKES_DIR/bin/gazsim.bash

for ((RUN=1 ; RUN<=$NUM_RUNS ;RUN++))
do
    for ((TEAM1=0 ; TEAM1<$NUM_TEAMS ;TEAM1++))
    do
	for ((TEAM2=$((1+$TEAM1)); TEAM2<$NUM_TEAMS ;TEAM2++))
	do
	    echo Executing simulation-run $RUN with ${TEAMS[$TEAM1]} vs. ${TEAMS[$TEAM2]}

	    #create and go to log folder
	    cd "$START_PATH"
	    MATCH_NAME=${TEAMS[$TEAM1]}-vs-${TEAMS[$TEAM2]}
	    mkdir -p "$MATCH_NAME/run_$RUN"
	    cd "$MATCH_NAME/run_$RUN"

	    #set config values for automated control
	    replace_config run $RUN
	    replace_config configuration-name "\"$MATCH_NAME\_run_$RUN\""
	    replace_config collection "\"$COMPETITION_NAME\_$TIME\""
	    export DIR_FOR_SED=$(echo $START_PATH/$MATCH_NAME/run_$RUN | sed "s/\//\\\\\//g") #creepy string because of sed
	    replace_config log "\"$DIR_FOR_SED\""

	    if $REPLAY
	    then
		REPLAY_PATH="$START_PATH/$MATCH_NAME/run_$RUN"
	        REPLAY=-e $REPLAY_PATH
	    else
	        REPLAY=
	    fi
	    echo $REPLAY
	    
	    # start simulation
	    echo Starting gazbeo
	    $STARTUP_SCRIPT_LOCATION -x start -n 0 -s $HEADLESS -c default $REPLAY
	    echo "Starting Team CYAN: ${TEAMS[$TEAM1]}"
	    $STARTUP_SCRIPT_LOCATION -x start -r -a -n ${NUMBER_ROBOTS[$TEAM1]} -s $HEADLESS -c default $REPLAY
	    echo "Starting Team MAGENTA: ${TEAMS[$TEAM2]}"
	    $STARTUP_SCRIPT_LOCATION -x start -r -a -n ${NUMBER_ROBOTS[$TEAM2]} -s $HEADLESS -c default -f 4 $REPLAY

	    ######DEBUG
	    exit 0                        

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
		    echo Something went wrong
		    echo Restarting run
		    echo Starting gazbeo
		    $STARTUP_SCRIPT_LOCATION -x start -n 0 -s $HEADLESS -c default $REPLAY
		    echo "Starting Team CYAN: ${TEAMS[$TEAM1]}"
		    $STARTUP_SCRIPT_LOCATION -x start -r -a -n ${NUMBER_ROBOTS[$TEAM1]} -s $HEADLESS -c default -o
		    echo "Starting Team MAGENTA ${TEAMS[$TEAM2]}"
		    $STARTUP_SCRIPT_LOCATION -x start -r -a -n ${NUMBER_ROBOTS[$TEAM2]} -s $HEADLESS -c default -f 4 -o
		    sleep 30
		fi
		
	    done
	done
	#wait until the record is stored
	sleep 10s
	restore_record "$REPLAY_PATH/state.log"
    done
done
