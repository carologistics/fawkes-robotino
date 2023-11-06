#!/bin/bash
# Automated experiment and evaluation on fawkes-robotino setups
# Generates a game layout and runs n experiments of an experimental
# setup and optionally a baseline. The resulting data is parsed and
# visualized using additional helper scripts.
# This script can be adapted for use with a specific experimental
# setup e.g. by including code that automatically swaps config
# files between baseline and experiment runs.
# Runs are conducted in an alternating fashion

refbox_args="--refbox-args \"--cfg-simulation simulation/fast_simulation.yaml --cfg-mps mps/mockup_mps.yaml --cfg-game game/game_1.yaml --cfg-mongodb mongodb/enable_mongodb.yaml\""
#central_args="-x start -o -k -r -n 1 --mongodb --central-agent m-central-clips-exec -m m-skill-sim --keep-tmpfiles"
central_args="-o -k -n 1 -m m-skill-sim --central-agent m-central-clips-exec"
#rl_central_args="-x start -o -k -r -n 1 --mongodb --central-agent m-central-clips-exec -m m-skill-sim --keep-tmpfiles"
rl_central_args="-o -k -n 1 -m m-skill-sim --central-agent m-central-clips-exec"
decentral_args="-r -k -o --mongodb -m m-distributed-skill-sim-clips-exec"
gazsim_path="$FAWKES_DIR/bin/./gazsim.bash"
scripts_path=$FAWKES_DIR/etc/scripts/eval_suite
rl_agent_path=$FAWKES_DIR/fawkes/src/plugins/clips-executive-rl/rl-agent 
rl_log_path=$FAWKES_DIR/fawkes/src/plugins/clips-executive-rl/rl-agent/sb3_log
rl_agent_name="MaskablePPO_RCLL_123"


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
   -n                Number of runs (games or trainings) (2x if baseline and experiment), default 3
   -t                Training mode on, default off
                     If the training mode is turned on only trainings are conducted

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
NUMBER_RUN=1
NUMBER_TRAININGS=1
EXPERIMENT_EVAL="" #"rl"
BASELINE_EVAL="central"
EXPERIMENT_REFBOX_ARGS=$refbox_args
BASELINE_REFBOX_ARGS=$refbox_args
LOAD_GAME="game_1.gz"
LOAD_AGENT=0
LOAD_AGENT_NAME="RCLL_RL_SA"
TRAINING_MODE=0
REFBOX_SPEED=4
GAME_TIME=$((1200/$REFBOX_SPEED))
GAMES_PER_TRAINING=10

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
      NUMBER_RUN=$2
      shift # past argument
      shift
      ;;
    -t)
      TRAINING_MODE=1
      shift # past argument
      shift
      ;;
    --load-game)
      LOAD_GAME=$2
      shift # past argument
      shift
      ;;
    --load-agent)
      LOAD_AGENT=1
      LOAD_AGENT_NAME=$2
      shift
      shift
      ;;
    --experiment)
      if [ "$2" = "central" ]; then
        EXPERIMENT_COMMAND="$gazsim_path $central_args $EXPERIMENT_REFBOX_ARGS"
        EXPERIMENT_EVAL="central"
      else
        if [ "$2" = "rl" ]; then
          EXPERIMENT_COMMAND="$gazsim_path $rl_central_args $EXPERIMENT_REFBOX_ARGS"
          EXPERIMENT_EVAL="rl"
        else
          if [ "$2" = "decentral" ]; then
            EXPERIMENT_COMMAND="$gazsim_path $decentral_args $EXPERIMENT_REFBOX_ARGS"
            EXPERIMENT_EVAL="decentral"
          else
            EXPERIMENT_COMMAND="$gazsim_path $2 $EXPERIMENT_REFBOX_ARGS"
          fi
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
        if [ "$2" = "rl" ]; then
          BASELINE_COMMAND="$gazsim_path $rl_central_args $BASELINE_REFBOX_ARGS"
          BASELINE_EVAL="rl"
        else
          if [ "$2" = "decentral" ]; then
            BASELINE_COMMAND="$gazsim_path $decentral_args $EXPERIMENT_REFBOX_ARGS"
            BASELINE_EVAL="decentral"
          else
            BASELINE_COMMAND="$gazsim_path $2 $EXPERIMENT_REFBOX_ARGS"
          fi
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

NUMBER_TRAININGS=$NUMBER_RUN

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
    gnome-terminal -- bash -i -c "$LLSF_REFBOX_DIR/bin/./llsf-refbox --cfg-simulation simulation/fast_simulation.yaml --cfg-mps mps/mockup_mps.yaml --cfg-mongodb mongodb/enable_mongodb.yaml"
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

rcll-loadgame () {      
$LLSF_REFBOX_DIR/bin/./restore_reports.bash $1 #rl_game_1.gz 
echo "Finished restore report starting refbox with loaded game"                                              
$FAWKES_DIR/bin/./gazsim.bash -x kill; 
#$FAWKES_DIR/bin/./gazsim.bash -o -k -n 1 -m m-skill-sim --central-agent m-central-clips-exec --refbox-args "--cfg-mps mps/mockup_mps.yaml --cfg-game game/game1.yaml --cfg-simulation simulation/fast_simulation.yaml --cfg-mongodb mongodb/enable_mongodb.yaml" "$@"
$FAWKES_DIR/bin/./gazsim.bash -o -k -n 2 -m m-skill-sim --central-agent m-central-clips-exec --refbox-args "--cfg-mps mps/mockup_mps.yaml --cfg-game game/game_1.yaml --cfg-simulation simulation/fast_simulation.yaml --cfg-mongodb mongodb/enable_mongodb.yaml"
} 

start_simulation () {
    echo "starting simulation $1"
    #eval $1
    rcll-loadgame $LOAD_GAME
    sleep 20
}

setup_simulation () {
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -w30
    #echo "Starting game (Phase: PRODUCTION; Cyan: Carologistics)"
    #$LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p SETUP -s RUNNING -c Carologistics
    #$LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -n 1 -W200 || (stop_simulation)
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p PRODUCTION -s RUNNING
}

setup_rl_simulation () {
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -w30
    #echo "Starting game (Phase: PRODUCTION; Cyan: Carologistics)"
    #$LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p SETUP -s RUNNING -c Carologistics
    #$LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -n 1 -W200 || (stop_simulation)
    echo "Starting game (Phase: PRODUCTION; Cyan: Carologistics)"
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p PRODUCTION -s RUNNING
}

wait_simulation_ends () {
    echo "Waiting for POST_GAME..."
    wait_time=$(($GAME_TIME+100))
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -W$wait_time -p POST_GAME
    sleep 60
    stop_simulation
}

wait_rl_training_ends () {
    echo "Waiting for POST_GAME..."
    training_time=$(($GAMES_PER_TRAINING*$GAME_TIME+10))
    echo $training_time
    sleep $training_time
    $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -W$training_time -p POST_GAME
    echo "Refbox in POST_GAME"
    sleep 60
    echo "Stopping simulation now"
    stop_simulation
}

activate_rl_agent_in_yaml()
{
    line=$(grep -n 'active' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    sed -i $line's/.*/  active: true/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

deactivate_rl_agent_in_yaml()
{
    line=$(grep -n 'active' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    sed -i $line's/.*/  active: false/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

set_rl_agent_training_mode_in_yaml()
{
    line=$(grep -n 'training-mode' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    sed -i $line's/.*/  training-mode: true/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

set_rl_agent_execution_mode_in_yaml()
{
    line=$(grep -n 'training-mode' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    sed -i $line's/.*/  training-mode: false/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

change_rl_agent_name_in_yaml()
{
    line=$(grep -n 'save-agent-name' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    echo ${line}
    sed -i $line's/.*/  save-agent-name: "'$rl_agent_name'"/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

set_load_agent_mode_in_yaml()
{
    line=$(grep -n 'load-agent:' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    sed -i $line's/.*/  load-agent: true/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

set_new_agent_mode_in_yaml()
{
    line=$(grep -n 'load-agent:' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    echo $line
    sed -i $line's/.*/  load-agent: false/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

change_load_agent_name_in_yaml()
{
    line=$(grep -n 'load-agent-name' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
    sed -i $line's/.*/  load-agent-name: "'$LOAD_AGENT_NAME'"/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml
}

copy_game_files () {
    mkdir `pwd`/$1/$2
    cp `pwd`/refbox_latest.log `pwd`/$1/$2/refbox.log
    cp `pwd`/refbox-debug_latest.log `pwd`/$1/$2/refbox-debug.log
    if [ "$3" = "central" ] || [ $TRAINING_MODE -eq 0 ]; then
        cp `pwd`/debug11_latest.log `pwd`/$1/$2/debug11.log
        python $scripts_path/parser.py --central TRUE --path `pwd`/$1/$2/ --postfix ".log"
        python $scripts_path/goal_visualizer.py --path `pwd`/$1/$2/
    fi
    if [ "$3" = "rl" ] && [ $TRAINING_MODE -eq 1 ]; then
        cp `pwd`/debug11_latest.log `pwd`/$1/$2/debug11.log
        python $scripts_path/trainings_parser_2.py --central TRUE --path `pwd`/$1/$2/ --postfix ".log" --ngames TRUE
        python $scripts_path/goal_visualizer.py --path `pwd`/$1/$2/ --ngames $GAMES_PER_TRAINING
        #python $scripts_path/rl_training_visualizer.py --path `pwd`/$1/$2/ --ngames $GAMES_PER_TRAINING
    fi
    if [ "$3" = "decentral" ]; then
        cp `pwd`/debug1_latest.log `pwd`/$1/$2/debug1.log
        cp `pwd`/debug2_latest.log `pwd`/$1/$2/debug2.log
        cp `pwd`/debug3_latest.log `pwd`/$1/$2/debug3.log
        python $scripts_path/parser.py --path `pwd`/$1/$2/ --postfix ".log"
        python $scripts_path/goal_visualizer.py --path `pwd`/$1/$2/
    fi

}

copy_rl_agent_files(){
  echo "Copy Game files " $rl_agent_path 
  echo $3

  cp $rl_agent_path/$3 `pwd`/$1/$2/$3
  cp $rl_log_path/log.txt `pwd`/$1/$2/log.txt
  cp $rl_log_path/progress.csv `pwd`/$1/$2/progress.csv
  cat `pwd`/.monitor.csv >> `pwd`/$1/$2/monitor.csv

  for i in $(seq $GAMES_PER_TRAINING)
  do
    echo log_stage3_facts_split_$i.log
    cp `pwd`/log_stage3_facts_split_$i.txt `pwd`/$1/$2/log_stage3_facts_split_$i.log
    cp `pwd`/log_stage4_facts_split_$i.txt `pwd`/$1/$2/log_stage4_facts_split_$i.log
    cp `pwd`/log_stage6_facts_split_$i.txt `pwd`/$1/$2/log_stage6_facts_split_$i.log
  done
}

run_simulation () {
    # name of directory, name of experiment directory, configuration, load or generate
    echo `pwd`/$1/$2/

    if [ "$4" = "rl" ]; then
      echo "Run agent with rl goal selection in execution mode!"
      change_rl_agent_name_in_yaml
      if [ $LOAD_AGENT -eq 0 ];
      then
        echo "- generating new agent = ${LOAD_AGENT}" >> $name/configuration.txt
        set_new_agent_mode_in_yaml
      else
        echo "- loading previous agent = ${LOAD_AGENT}" >> $name/configuration.txt
        set_load_agent_mode_in_yaml
        change_load_agent_name_in_yaml
      fi
      activate_rl_agent_in_yaml
      set_rl_agent_execution_mode_in_yaml
      #line=$(grep -n 'Maskable' $FAWKES_DIR/cfg/conf.d/rl-test.yaml | cut -d ':' -f1)
      #sed -i $line's/.*/  name: "'$rl_agent_name'"/' $FAWKES_DIR/cfg/conf.d/rl-test.yaml

      start_simulation "$3"
      setup_rl_simulation
    else
      deactivate_rl_agent_in_yaml
      set_rl_agent_execution_mode_in_yaml

      start_simulation "$3"
      setup_simulation
    fi
    wait_simulation_ends
    copy_game_files $1 $2 $4
    #if [ "$3" = "rl" ]; then
    #  copy_rl_agent_files $1 $2 $rl_agent_name
    #fi
}

run_rl_training () {
    # name of directory, name of experiment directory, configuration, load or generate

    echo "Run rl training!!!"
    echo `pwd`/$1/$2/
    if [ $LOAD_AGENT -eq 0 ];
    then
      echo "test"
      echo "- generating new agent = ${LOAD_AGENT}"
      set_new_agent_mode_in_yaml
    else
      echo "- loading previous agent = ${LOAD_AGENT}"
      set_load_agent_mode_in_yaml
      change_load_agent_name_in_yaml
    fi
    change_rl_agent_name_in_yaml
    activate_rl_agent_in_yaml
    set_rl_agent_training_mode_in_yaml
    start_simulation "$3"

    setup_rl_simulation
    wait_rl_training_ends
    copy_game_files $1 $2 $4
    copy_rl_agent_files $1 $2 $rl_agent_name
    
}


generate_name () {
    date +"simulation_%y-%m-%d_%H-%M"
}

generate_agent_name () {
    date +"MaskablePPO_RCLL_%y-%m-%d_%H-%M.zip"
}

#generate experiment dictionary
name=`generate_name`
mkdir $name


#writeout configuration
echo "Your chosen configuration: " >> $name/configuration.txt
echo "- number of runs      = ${NUMBER_RUN}" >> $name/configuration.txt
echo "- custom mongo config = ${CUSTOM_MONGO}" >> $name/configuration.txt
echo "- experiment command  = ${EXPERIMENT_COMMAND}" >> $name/configuration.txt
echo "- experiment eval     = ${EXPERIMENT_EVAL}" >> $name/configuration.txt
echo "- load game           = ${LOAD_GAME}" >> $name/configuration.txt
echo "- refbox speedup = ${REFBOX_SPEED}" >> $name/configuration.txt

if [ $TRAINING_MODE -eq 0 ];
then
  echo "- trainings mode off  = ${TRAINING_MODE}" >> $name/configuration.txt
  if [ -z ${BASELINE_COMMAND+x} ];
  then
      echo "- baseline command    = not set, only doing experiment runs"; >> $name/configuration.txt
  else
      echo "- baseline command    = ${BASELINE_COMMAND}" >> $name/configuration.txt
      echo "- baseline eval       = ${BASELINE_EVAL}" >> $name/configuration.txt
  fi
else
  rl_agent_name=`generate_agent_name`
  echo "- trainings mode on  = ${TRAINING_MODE}" >> $name/configuration.txt
  echo "- number training = ${NUMBER_TRAININGS}" >> $name/configuration.txt
  echo "- number of games per training = ${GAMES_PER_TRAINING}" >> $name/configuration.txt
fi
echo "- agent name          = ${rl_agent_name}" >> $name/configuration.txt

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


#if [ -z ${LOAD_GAME+x}];
#then
  #start a simulation and let it run for a view seconds to generate a valid configuration to load from
  #echo "Start a refbox to generate a game configuration"
  #start_simulation_generate_game
#fi

if [ $TRAINING_MODE -eq 0 ]; then
  for k in $(seq $NUMBER_RUN)
  do
      echo "Training mode off!"
      #run experiment
      run_simulation $name experiment$k "$EXPERIMENT_COMMAND" $EXPERIMENT_EVAL
      dir_monitoring+=" experiment$k"
      #if baseline configured, run baseline
      if ! [ -z ${BASELINE_COMMAND+x} ]; then
        run_simulation $name baseline$k "$BASELINE_COMMAND" $BASELINE_EVAL
      fi
  done
else
  dir_monitoring=""
  #start simulation (with fawkes) $NUMBER_TRAININGS times
  for k in $(seq $NUMBER_TRAININGS)
  do
      #run training
      run_rl_training $name training$k "$EXPERIMENT_COMMAND" $EXPERIMENT_EVAL
      dir_monitoring+=" training$k"
  done
  echo "Monitoring dirs: " $dir_monitoring
  echo "path: " `pwd`/$name
  python $scripts_path/visualize_monitoring.py --path `pwd`/$name --name monitor.csv --dirs $dir_monitoring
fi

#if more than 2 games and baseline+experiment, visualize
if ! [ -z ${BASELINE_COMMAND+x} ] && [ "$NUMBER_RUN" -gt "1" ]; then
  python $scripts_path/exp_visualizer.py --path $name
fi

#clean up simulation
stop_mongod
cleanup_gazsim
