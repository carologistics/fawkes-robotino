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


# Only run the simtest on Fedora.
source /etc/os-release
if [ "$NAME" != "Fedora" ] ; then
  exit 0
fi

sed -i 's_simtest/enabled:.*_simtest/enabled: true_' cfg/conf.d/*
SCRIPT_PATH=$PWD/bin/
pushd $SCRIPT_PATH
TERMINAL=tmux
export TERMINAL
$SCRIPT_PATH/gazsim.bash -o -r -m m-skill-sim-clips-exec -k -n 3 --team-cyan Carologistics --start-game=PRODUCTION
$SCRIPT_PATH/cx-simtest-check.bash ./robot1_latest.log ./robot2_latest.log ./robot3_latest.log
$SCRIPT_PATH/gazsim.bash -x kill
popd
