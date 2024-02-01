#!/bin/bash
# Automated experiment and evaluation on fawkes-robotino setups
# Generates a game layout and runs n experiments of an experimental
# setup and optionally a baseline. The resulting data is parsed and
# visualized using additional helper scripts.
# This script can be adapted for use with a specific experimental
# setup e.g. by including code that automatically swaps config
# files between baseline and experiment runs.
# Runs are conducted in an alternating fashion

# refbox_args="--refbox-args \"--cfg-simulation simulation/default_simulation.yaml --cfg-mps mps/mockup_mps.yaml --cfg-game game/load_game.yaml --cfg-mongodb mongodb/enable_mongodb.yaml\""
# central_args="-x start -o -k -r -n 3 --mongodb --central-agent m-central-clips-exec -m m-skill-sim --keep-tmpfiles"
central_args="-o -k -n 3 --mongodb -m m-skill-sim --central-agent m-central-clips-exec"
refbox_args="--refbox-args \"--cfg-mps mps/mockup_mps.yaml\"" 

decentral_args="-r -k -o --mongodb -m m-distributed-skill-sim-clips-exec"
gazsim_path="$FAWKES_DIR/bin/./gazsim.bash"
scripts_path=$FAWKES_DIR/etc/scripts/eval_suite

usage()
{
cat << EOF
usage: $0 options

This script can be used to automatically conduct experiments of
a fawkes-robotino setup and optionally compare it to a baseline
on the same setup. The data is parsed and visualized for easier
evaluation and inspection using helper scripts.

Base Options:
   -h                Show this message
   -m                Use custom ports for mongodb (requires config change)
                     and launch the instances.
                     The custom ports are: 27019 for refbox and 27018 for agents.
   -n                Number of games (2x if baseline and experiment), default 3

Experiment and baseline configurations:
       experiment defines the setup to be tested whereas baseline defines
       an alternative setup that should be compared against. By default
       (with no additional arguments) the central agent is run n times.
       In any case, a refbox instance is started first to generate a field.
       Therefore, the given startup commands for gazsim.bash should configure the
       refbox s.t. it loads the last game-report.
   --experiment "<str>" Command string to start an experiment run with gazsim.bash
   --baseline   "<str>" Command string to start a baseline run with gazsim.bash
       By supplying the argument "central" or "decentral", the central or decentral
       agent is started in standard configuration and the corresponding evaluation
       mode is used. Alternativaly, arguments for gazsim.bash can be supplied to
       create a custom configuration. In all cases, the refbox is configured
       to use mockup mps, load the game, use mongodb and run with default
       simulation settings. These following options can be used to overwrite
       the refbox arguments for baseline and experiment. If the refbox modifiers
       are used, they must be entered before the experiment and baseline configurations
       appear in the argument list.
   --experiment-refbox "<str>" arguments for a custom refbox configuration.
   --baseline-refbox "<str>" arguments for a custom refbox configuration.

       If instead a different simulation script should be used, the following options
       can be used to define an alternative executable path. However, in this case all
       arguments must be provided directly and the refbox has the be started by the given
       script.
   --experiment-path "<str>" custom executable path for starting a simulation environment.
   --baseline-path "<str>" custom executable path for starting a simulation environment.

Evaluation modes for experiment and baseline runs to pass through to parser.
This is required because the decentral agent produces m logs, for the number
of agents configured whereas the central agent produces only one, differently
labelled log. The possible values are "central" and "decentral". By default,
"central" is used.
   --experiment-eval   Evaluation mode of experiment runs
   --baseline-eval     Evaluation mode of baseline runs
EOF
}


POSITIONAL_ARGS=()

CUSTOM_MONGO=0
NUMBER_GAMES=3
EXPERIMENT_EVAL="central"
BASELINE_EVAL="central"
EXPERIMENT_REFBOX_ARGS=$refbox_args
BASELINE_REFBOX_ARGS=$refbox_args

while [[ $# -gt 0 ]]; do
  case $1 in
    -h|--help)
      shift # past argument
      usage
      exit 1
      ;;
    -m)
      CUSTOM_MONGO=1
      shift # past argument
      ;;
    -n)
      NUMBER_GAMES=$2
      shift # past argument
      shift
      ;;
    --experiment)
      if [ "$2" = "central" ]; then
        EXPERIMENT_COMMAND="$gazsim_path $central_args $EXPERIMENT_REFBOX_ARGS"
        EXPERIMENT_EVAL="central"
      else
        if [ "$2" = "decentral" ]; then
          EXPERIMENT_COMMAND="$gazsim_path $decentral_args $EXPERIMENT_REFBOX_ARGS"
          EXPERIMENT_EVAL="decentral"
        else
          EXPERIMENT_COMMAND="$gazsim_path $2 $EXPERIMENT_REFBOX_ARGS"
        fi
      fi
      shift # past argument
      shift
      ;;
    --baseline)
      if [ "$2" = "central" ]; then
        BASELINE_COMMAND="$gazsim_path $central_args $BASELINE_REFBOX_ARGS"
        BASELINE_EVAL="central"
      else
        if [ "$2" = "decentral" ]; then
          BASELINE_COMMAND="$gazsim_path $decentral_args $BASELINE_REFBOX_ARGS"
          BASELINE_EVAL="decentral"
        else
          BASELINE_COMMAND="$gazsim_path $2 $BASELINE_REFBOX_ARGS"
        fi
      fi
      shift # past argument
      shift
      ;;
    --experiment-refbox)
      EXPERIMENT_REFBOX_ARGS=$2
      shift # past argument
      shift
      ;;
    --experiment-path)
      EXPERIMENT_COMMAND="$2"
      shift # past argument
      shift
      ;;
    --baseline-refbox)
      BASELINE_REFBOX_ARGS=$2
      shift # past argument
      shift
      ;;
    --baseline-path)
      BASELINE_COMMAND=$2
      shift # past argument
      shift
      ;;
    --experiment-eval)
      EXPERIMENT_EVAL=$2
      shift # past argument
      ;;
    --baseline-eval)
      BASELINE_EVAL=$2
      shift # past argument
      ;;
    -*|--*)
      echo "Unknown option $1"
      usage
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1") # save positional arg
      shift # past argument
      ;;
  esac
done

stop_simulation () {
    killall gazsim-startup.
    killall gzserver &> /dev/null
    killall gzclient &> /dev/null
    killall fawkes  &> /dev/null
    killall roscore &> /dev/null
    killall llsf-refbox  &> /dev/null
    killall llsf-refbox-shell &> /dev/null
    killall roslaunch &> /dev/null
    pkill -9 -f gazsim-startup.bash &> /dev/null
    pkill -9 -f "Waiting for shutdown" &> /dev/null
    sleep 5
}

stop_mongod () {
    killall mongod
}

stop_mongod_robot_instances () {
    kill $(ps aux | grep 'mongod.*27018' | head -2  | awk '{print $2}')
}


stop_mongod_robot_instances () {
    kill $(ps aux | grep 'mongod.*27018' | head -2  | awk '{print $2}')
}


cleanup_gazsim () {
    rm -r /tmp/gazsim*
}

start_refbox_mongod_instance () {
    MONGODB_DBPATH=$(mktemp -d --tmpdir mongodb-27019-XXXXXXXXXXXX)
    gnome-terminal -- bash -i -c "mongod --port 27019 --dbpath $MONGODB_DBPATH | tee mongodb.log"
    sleep 5
}

start_agents_mongod_instance () {
    MONGODB_DBPATH=$(mktemp -d --tmpdir mongodb-27018-XXXXXXXXXXXX)
    gnome-terminal -- bash -i -c "mongod --port 27018 --dbpath $MONGODB_DBPATH | tee mongodb.log"
    sleep 5
}

start_simulation_generate_game() {
    gnome-terminal -- bash -i -c "$LLSF_REFBOX_DIR/bin/./llsf-refbox --cfg-simulation simulation/default_simulation.yaml --cfg-mps mps/mockup_mps.yaml --cfg-mongodb mongodb/enable_mongodb.yaml"
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -w20
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p SETUP -s RUNNING -c Carologistics
    sleep 2
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p PRODUCTION -s RUNNING
    sleep 2
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p POST_GAME
    sleep 2
    killall llsf-refbox
    sleep 2
    killall llsf-refbox
    sleep 2
}

start_simulation () {
    echo "starting simulation $1"
    # eval $1
    $FAWKES_DIR/bin/./gazsim.bash -x kill;
    $FAWKES_DIR/bin/./gazsim.bash -o -k -n 3 --mongodb -m m-skill-sim --central-agent m-central-clips-exec --refbox-args "--cfg-mps mps/mockup_mps.yaml"
    sleep 20
}

setup_simulation () {
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -w30
    echo "Starting game (Phase: PRODUCTION; Cyan: Carologistics)"
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p SETUP -s RUNNING -c Carologistics
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -n 3 -W200 || (stop_simulation)
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p PRODUCTION -s RUNNING
}

wait_simulation_ends () {
    echo "Waiting for POST_GAME..."
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -W1300 -p POST_GAME
    sleep 60
    stop_simulation
}

copy_game_files () {
    mkdir `pwd`/$1/$2
    cp `pwd`/refbox_latest.log `pwd`/$1/$2/refbox.log
    cp `pwd`/refbox-debug_latest.log `pwd`/$1/$2/refbox-debug.log
    if [ "$3" = "central" ]; then
        cp `pwd`/debug11_latest.log `pwd`/$1/$2/debug11.log
        python $scripts_path/parser.py --central TRUE --path `pwd`/$1/$2/ --postfix ".log"
    fi
    if [ "$3" = "decentral" ]; then
        cp `pwd`/debug1_latest.log `pwd`/$1/$2/debug1.log
        cp `pwd`/debug2_latest.log `pwd`/$1/$2/debug2.log
        cp `pwd`/debug3_latest.log `pwd`/$1/$2/debug3.log
        python $scripts_path/parser.py --path `pwd`/$1/$2/ --postfix ".log"
    fi

    python $scripts_path/goal_visualizer.py --path `pwd`/$1/$2/
}

run_simulation () {
    # name of directory, name of experiment directory, configuration, load or generate
    echo `pwd`/$1/$2/
    start_simulation "$3"
    setup_simulation
    wait_simulation_ends
    copy_game_files $1 $2 $4
}

generate_name () {
    date +"simulation_%y-%m-%d_%H-%M"
}

#generate experiment dictionary
name=`generate_name`
mkdir $name

#writeout configuration
echo "Your chosen configuration: " >> $name/configuration.txt
echo "- number of games     = ${NUMBER_GAMES}" >> $name/configuration.txt
echo "- custom mongo config = ${CUSTOM_MONGO}" >> $name/configuration.txt
echo "- experiment command  = ${EXPERIMENT_COMMAND}" >> $name/configuration.txt
echo "- experiment eval     = ${EXPERIMENT_EVAL}" >> $name/configuration.txt
if [ -z ${BASELINE_COMMAND+x} ];
then
    echo "- baseline command    = not set, only doing experiment runs"; >> $name/configuration.txt
else
    echo "- baseline command    = ${BASELINE_COMMAND}" >> $name/configuration.txt
    echo "- baseline eval       = ${BASELINE_EVAL}" >> $name/configuration.txt
fi

cat $name/configuration.txt

#clean up previous simulation leftovers
cleanup_gazsim
stop_simulation

if [ $CUSTOM_MONGO -eq 1 ]; then
    #start mongod instance for refbox
    echo "Starting MongoDB instance on port 27019 for refbox"
    start_refbox_mongod_instance

    #start central mongod instance for robots and generate skill times
    echo "Starting MongoDB instance on port 27018"
    start_agents_mongod_instance
fi

#start a simulation and let it run for a view seconds to generate a valid configuration to load from
echo "Start a refbox to generate a game configuration"
start_simulation_generate_game

#run simulation $NUMBER_GAMES times
for i in $(seq $NUMBER_GAMES)
do
    #run experiment
    run_simulation $name experiment$i "$EXPERIMENT_COMMAND" $EXPERIMENT_EVAL
    #if baseline configured, run baseline
    if ! [ -z ${BASELINE_COMMAND+x} ]; then
      run_simulation $name baseline$i "$BASELINE_COMMAND" $BASELINE_EVAL
    fi
done

#if more than 2 games and baseline+experiment, visualize
if ! [ -z ${BASELINE_COMMAND+x} ] && [ "$NUMBER_GAMES" -gt "1" ]; then
  python $scripts_path/exp_visualizer.py --path $name
fi

#clean up simulation
stop_mongod
cleanup_gazsim
