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

FAWKES_DIR=$(realpath $(dirname ${BASH_SOURCE[0]})/..)
export FAWKES_DIR
SCRIPT_PATH=$FAWKES_DIR/bin

TRAP_SIGNALS="SIGINT SIGTERM SIGPIPE EXIT"
stop_test () {
  trap - $TRAP_SIGNALS
  $SCRIPT_PATH/cx-central-simtest-challenge-wrapper.bash kill >/dev/null
}

echo "Starting with C0 production test"
trap stop_test $TRAP_SIGNALS
ulimit -c 0
$SCRIPT_PATH/cx-central-simtest-challenge-wrapper.bash --C0-production $@
