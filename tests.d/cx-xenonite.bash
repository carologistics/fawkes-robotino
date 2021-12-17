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


FAWKES_DIR=$(realpath $(dirname ${BASH_SOURCE[0]})/..)
tmpconfig=$(mktemp $FAWKES_DIR/cfg/conf.d/simtest-XXXXXX.yaml)
echo "---" >> $tmpconfig
echo "/clips-executive/spec: xenonite" >> $tmpconfig
export FAWKES_DIR
SCRIPT_PATH=$FAWKES_DIR/bin
WORKING_DIR=$FAWKES_DIR/tests.out.d/cx-xenonite
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
$SCRIPT_PATH/gazsim.bash -x start -k -n 3 -k -o --no-refbox --no-refbox-comm --mongodb -m m-promises $@
sleep 10
echo "Waiting for results..."
timeout -k 390 360 $SCRIPT_PATH/cx-xenonite-check.bash ./robot1_latest.log ./robot2_latest.log ./robot3_latest.log
res=$?
if [ "$res" -eq 0 ] ; then
  echo "Test succeeded."
else
  echo "Test failed."
fi
exit $res
