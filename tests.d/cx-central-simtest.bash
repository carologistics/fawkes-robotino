#! /bin/bash
#
# cx-central-simtest.bash
# Copyright (C) 2020 Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

if [ -n $LLSF_REFBOX_DIR ] ; then
    export PATH=$LLSF_REFBOX_DIR:$PATH
fi


FAWKES_DIR=$(realpath $(dirname ${BASH_SOURCE[0]})/..)
tmpconfig=$(mktemp $FAWKES_DIR/cfg/conf.d/simtest-XXXXXX.yaml)
tmprefconfig=$(mktemp $LLSF_REFBOX_DIR/cfg/simtest-XXXXXX.yaml)

echo "/clips-executive/specs/rcll-central/parameters/simtest/enabled: true" > $tmpconfig
echo "/clips-executive/specs/rcll-central/parameters/simtest/testbed: FULL" >> $tmpconfig
echo "/clips-executive/spec: rcll-central" >> $tmpconfig
echo "/plugins/execution-time-estimator/static/speed/: 2" >> $tmpconfig
echo "/plugins/execution-time-estimator/navgraph/speed/: 1" >> $tmpconfig

echo "/llsfrb/game/random-machine-down-times: false" > $tmprefconfig
echo "/llsfrb/simulation/enable: true" >> $tmprefconfig
echo "/llsfrb/simulation/speedup: 2" >> $tmprefconfig
echo "/llsfrb/simulation/time-sync/enable: false" >> $tmprefconfig
echo "/llsfrb/simulation/time-sync/estimate-time: false" >> $tmprefconfig

export FAWKES_DIR
SCRIPT_PATH=$FAWKES_DIR/bin
WORKING_DIR=$FAWKES_DIR/tests.out.d/cx-central-simtest
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
echo "$tmprefconfig"

$SCRIPT_PATH/gazsim.bash -o -r --mongodb \
  -m m-skill-sim --central-agent m-central-clips-exec -n 3 \
  --team-cyan Carologistics --start-game=PRODUCTION \
  --refbox-args "--cfg-mps mps/mockup_mps.yaml --cfg-custom $tmprefconfig" \
  $@
echo "Waiting for results..."
$SCRIPT_PATH/cx-simtest-check.bash ./robot11_latest.log
