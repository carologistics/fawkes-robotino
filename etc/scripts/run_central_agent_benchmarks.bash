#!/bin/bash
# Automatically starts games from the rcll-refbox benchmark set "rcll100",
# requres this set to be loaded to the database."

echo "usage: ./run_central_agent_benchmarks <benchmark_name> <num_games_to_run> <offset>"

function start_simulation {
  pwd=$(pwd)
  cd $1/logdump
  current_game=$(sed -r "s/load-from-report.*/load-from-report: \"$2\"/g;s/store-to-report.*/store-to-report: \"$3\"/g" $LLSF_REFBOX_DIR/cfg/game/load_game.yaml)
  echo "$current_game" > game_generated.yaml
  $FAWKES_DIR/bin/./gazsim.bash -x kill; \
  $FAWKES_DIR/bin/./gazsim.bash -o -r -k --mongodb -m m-skill-sim \
    --central-agent m-central-clips-exec \
    --refbox-args "--cfg-mps mps/mockup_mps.yaml \
       --cfg-mongodb mongodb/enable_mongodb.yaml \
       --cfg-game $pwd/$1/logdump/game_generated.yaml \
       --cfg-simulation simulation/fast_simulation.yaml \
       --dump-cfg"
  cd $pwd
}

function stop_simulation {
  killall gzserver &>/dev/null
  killall gzclient &>/dev/null
  killall fawkes &>/dev/null
  killall roscore &>/dev/null
  killall llsf-refbox &>/dev/null
  killall llsf-refbox-shell &>/dev/null
  killall roslaunch &>/dev/null
  pkill -9 -f gazsim-startup.bash &>/dev/null
  pkill -9 -f "Waiting for shutdown" &>/dev/null
  sleep 5
}

function run_simulation() {
  #start the simulation
  echo "Starting simulation $1"
  start_simulation $2 $3 $1;sleep 10

  #instruct startup
  $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -w30
  echo "Starting game (Phase: PRODUCTION; Cyan: Carologistics)"
  $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p SETUP -s RUNNING -c Carologistics
  $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -n 3 -W200 || (stop_simulation)
  $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -p PRODUCTION -s RUNNING

  #waiting for game to end
  echo "Waiting for POST_GAME..."
  $LLSF_REFBOX_DIR/bin/./rcll-refbox-instruct -W1200 -p POST_GAME
  sleep 5

  #stop the simulation processes
  echo "Stopping simulation"
  stop_simulation
}

function copy_logs() {
  #copy the logfiles
  echo "Copying logs"
  mkdir $1/$2
  path=$1/$2/${2}
  cp $1/logdump/debug11_latest.log ${path}_central_agent.log
  cp $1/logdump/refbox_latest.log ${path}_refbox.log
  cp $1/logdump/refbox-debug_latest.log ${path}_refbox-debug.log
  touch $1/$2/$3
  sleep 2
}

function delete_tmp_files() {
  rm -rfd $FAWKES_DIR/tests.out.d/cx-simtest/*
  rm -rfd /tmp/gazsim-*
  #rm $1/logdump/d* $1/logdump/r* $1/logdump/m* -r
}

#name args
arg_target_dir=$1
arg_runs=$(( $2 + 1 ))

#create the directory if it doesn't exist
if [ -d "$arg_target_dir" ]; then
  echo "Given directory exists"
else
  echo "Creating directory"
  mkdir -p $arg_target_dir
  mkdir -p $arg_target_dir/logdump
fi

#create tmpconfig
#tmpconfig=$(mktemp $FAWKES_DIR/cfg/conf.d/simtest-XXXXXX.yaml)
#echo "/clips-executive/specs/rcll/parameters/simtest/enabled: true" > $tmpconfig
#echo "/clips-executive/spec: rcll" >> $tmpconfig

#perform test arg_runs-times
count=1
retries=0
failed_runs=0
failed_benchmark=""

if [ "$3" != "" ]; then
  count=$3
  echo "Starting at $3"
fi

reports_list=""

while [ $arg_runs -gt $count ]
do
  #create game params
  currdate=$(date +%Y-%m-%d_%H-%M-%S)
  gamereport=$(echo "benchmark_$count")
  dirname=$(echo "$gamereport-$currdate")

  echo "----------------------------------------------------------------------"
  echo "Run $count/$((arg_runs - 1)) starting!"
  echo "Using benchmark gamereport $gamereport"
  echo "Files saved at $dirname"

  #prepare refbox settings

  #run the simulation
  run_simulation $dirname $arg_target_dir $gamereport

  #copy log files
  copy_logs $arg_target_dir $dirname $gamereport

  #quick-check produced data
  echo "Verify Game Length"
  outcome=$(python $LLSF_REFBOX_DIR/etc/scripts/extract_game_report_data.py --report-names "$dirname" --skip-short)
  echo "python $LLSF_REFBOX_DIR/etc/scripts/extract_game_report_data.py --report-names $dirname --skip-short)"
  if [[ $outcome == *"'Carologistics': 1"* ]]; then
    echo "Game length matches"
    reports_list="$reports_list $dirname"
    count=$((count + 1 ))
    retries=0
  else
    if [ $retries -ge 10 ]; then
      echo "Max number of retries reached, stopping!"
      failed_benchmark="benchmark_$count"
      count=$((count + arg_runs))
    else
      echo "Retrying!"
      failed_runs=$((failed_runs + 1 ))
      retries=$((retries + 1))
      rm -r $arg_target_dir/$dirname
    fi
  fi
  #cleanup and restoring the previous refbox settings
  delete_tmp_files $arg_target_dir
done

echo "========================================================================"

successful_benchmarks=$(echo "$reports_list" | tr -cd ' \t' | wc -c)
#count the number of results
echo "Evaluate Results ($successful_benchmarks successful, $failed_runs retries)"

if [[ $failed_benchmark != "" ]]; then
echo "Failed at $failed_benchmark"
fi
if [[ $successful_benchmarks -gt 0 ]]; then
  echo "The following reports are used: $reports_list"
  pwd=$(pwd)
  cd $arg_target_dir
  python $LLSF_REFBOX_DIR/etc/scripts/extract_game_report_data.py --report-names $reports_list --boxplots --boxplot-tables --accumulated-bar-diagrams --export-csv
  cd $pwd
fi
