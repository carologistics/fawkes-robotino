#! /bin/bash
#
# cx-simtest.bash
# Copyright (C) 2019 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# Read the full text in the LICENSE.GPL file in the doc directory.
#

set -u -o pipefail

# Only run the simtest on Fedora.
source /etc/os-release
if [ "$ID" != "fedora" ] ; then
  exit 0
fi

FAWKES_DIR=$(realpath $(dirname ${BASH_SOURCE[0]})/..)
tmpconfig=$(mktemp $FAWKES_DIR/cfg/conf.d/simtest-XXXXXX.yaml)
echo "/clips-executive/specs/rcll/parameters/simtest/enabled: true" > $tmpconfig
echo "/clips-executive/specs/rcll/parameters/simtest/testbed: BUILDTEST" >> $tmpconfig
echo "/plugins/execution-time-estimator/static/speed: 4" >> $tmpconfig
echo "/plugins/execution-time-estimator/navgraph/speed: 2" >> $tmpconfig
echo "/plugins/execution-time-estimator/lookup/speed: 4.0" >> $tmpconfig
export FAWKES_DIR
SCRIPT_PATH=$FAWKES_DIR/bin
WORKING_DIR=$FAWKES_DIR/tests.out.d/cx-simtest
mkdir -p $WORKING_DIR
cd $WORKING_DIR
TERMINAL=tmux
export TERMINAL

TRAP_SIGNALS="SIGINT SIGTERM SIGPIPE EXIT"
stop_test () {
  trap - $TRAP_SIGNALS
  $SCRIPT_PATH/gazsim.bash -x kill >/dev/null
  rm -f $tmpconfig
}

trap stop_test $TRAP_SIGNALS
ulimit -c 0
if timeout 120 $SCRIPT_PATH/gazsim.bash -o --mongodb \
  -m m-distributed-skill-sim-clips-exec -n 3 \
  --team-cyan Carologistics --start-game=PRODUCTION \
  --refbox-args "--cfg-mps mps/mockup_mps.yaml\
     --cfg-simulation simulation/fast_simulation.yaml \
     --cfg-game game/buildtest_game.yaml"
then
  echo "Waiting for results..."
  if timeout 360 $SCRIPT_PATH/cx-simtest-check.bash ./robot1_latest.log ./robot2_latest.log ./robot3_latest.log
  then
    echo "Simtest over."
    exit 0
  else
    echo "Failure: Timeout during execution."
    exit 1
  fi
else
  echo "Failure: Timeout during startup."
  exit 1
fi
