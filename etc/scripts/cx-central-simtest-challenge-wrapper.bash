#! /bin/bash
#
# cx-central-simtest-challenge-wrapper.bash
# Copyright (C) 2021 Sonja Ginter <sonja.ginter@rwth-aachen.de>
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Library General Public License for more details.
#
# Read the full text in the LICENSE.GPL file in the doc directory.
#
usage()
{
cat << EOF
usage: $0 <options>
Options:
   -h				Show this message

Tests:				Runs the specified test
   --enter-field
   --C0-production
   --C3-production
   --pick-and-place
EOF
}

set -eu -o pipefail

# Only run the simtest on Fedora.
source /etc/os-release
if [ "$NAME" != "Fedora" ] ; then
  exit 0
fi

FAWKES_DIR=$(realpath $(dirname ${BASH_SOURCE[0]})/..)
tmpconfig=$(mktemp $FAWKES_DIR/cfg/conf.d/simtest-XXXXXX.yaml)
specs_path=/clips-executive/specs/rcll-central/parameters
testbed_path=$specs_path/simtest/testbed:
echo "$specs_path/simtest/enabled: true" >> $tmpconfig
ref_args=
n=1

OPTS=$(getopt -o "h" -l "enter-field,C0-production,C3-production,pick-and-place" -- "$@")

if [ $? != 0 ]
then
	echo "Failed to parse parameters"
	usage
	exit 1
fi

eval set -- "$OPTS"
while true; do
	OPTION=$1
	case $OPTION in
		-h)
			usage
			rm -f $tmpconfig
			exit 1
			;;
		--enter-field)
			echo "$testbed_path ENTER-FIELD" >> $tmpconfig
			ref_args="--production c0 --ground-truth"
			shift
			break
			;;
		--C0-production)
			echo "$testbed_path C0-PRODUCTION" >> $tmpconfig
			ref_args="--production c0 --ground-truth"
			shift
			break
			;;
		--C3-production)
			echo "$testbed_path C3-PRODUCTION" >> $tmpconfig
			ref_args="--production c3 --ground-truth"
			shift
			break
			;;
		--pick-and-place)
			echo "$testbed_path PICK-AND-PLACE" >> $tmpconfig
			echo "$specs_path/rcll/pick-and-place-challenge: true" >> $tmpconfig
			ref_args="--grasping -- --cfg-game game/default_game.yaml"
			n=3
			shift
			break
			;;
		--)
			#default value
			echo "Default enter-field test before a c0 production"
			echo "$testbed_path ENTER-FIELD" >> $tmpconfig
			ref_args="--production c0"
			shift
			break
			;;
		esac
		shift
done

echo "Simtest: calling refbox with options: " $ref_args
echo "/clips-executive/spec: rcll-central" >> $tmpconfig
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

$FAWKES_DIR/bin/./gazsim.bash -x kill;
trap stop_test $TRAP_SIGNALS
ulimit -c 0
$SCRIPT_PATH/gazsim.bash -k -o -r --mongodb -m m-skill-sim --central-agent m-central-clips-exec -n $n --team-cyan Carologistics --challenge --refbox-args "--dump-cfg $ref_args" --start-game=PRODUCTION $@
echo "Waiting for results..."
$SCRIPT_PATH/cx-simtest-check.bash ./robot11_latest.log
