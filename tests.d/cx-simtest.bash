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

set -eu -o pipefail

# Only run the simtest on Fedora.
source /etc/os-release
if [ "$NAME" != "Fedora" ] ; then
  exit 0
fi


FAWKES_DIR=$PWD
tmpconfig=$(mktemp $FAWKES_DIR/cfg/conf.d/simtest-XXXXXX.yaml)
echo "/clips-executive/specs/rcll/parameters/simtest/enabled: true" > $tmpconfig
export FAWKES_DIR
SCRIPT_PATH=$FAWKES_DIR/bin/
WORKING_DIR=$FAWKES_DIR/tests.out.d/cx-simtest
mkdir -p $WORKING_DIR
cd $WORKING_DIR
TERMINAL=tmux
export TERMINAL
ROS_LOG_DIR=$WORKING_DIR/ros
export ROS_LOG_DIR

stop_test () {
  $SCRIPT_PATH/gazsim.bash -x kill
  rm -f $tmpconfig
}

trap "echo Aborting simulation test; stop_test" SIGINT SIGTERM SIGPIPE EXIT
ulimit -c 0
$SCRIPT_PATH/gazsim.bash -o -r --mongodb -m m-skill-sim-clips-exec -n 3 --team-cyan Carologistics --start-game=PRODUCTION $@
echo "Waiting for results..."
$SCRIPT_PATH/cx-simtest-check.bash ./robot1_latest.log ./robot2_latest.log ./robot3_latest.log
stop_test
# Workaround for https://github.com/buildkite/agent/issues/1203
rm -f $ROS_LOG_DIR/latest
