#!/bin/bash
# Automated startup of a Gazebo simulation
#
# Copyright (C) 2021 Tarik Viehmann <viehmann@kbsg.rwth-aachen.de>
#
CFG_DIR=$FAWKES_DIR/cfg/host_off-field1.d
ACTIVE_ROBOTS=$@

if [ -z "$ACTIVE_ROBOTS" ]
  then
		ACTIVE_ROBOTS=$(seq 1 3)
fi
ALL_ROBOTS=$(seq 1 3)

for CURR_ROBO in $ACTIVE_ROBOTS
do
		[[ $CURR_ROBO =~ ^[0-9]+$ ]] || {
			echo "Enter the robots that are active (out of [1, 2, 3])";
			exit 1;
		}
		if (($CURR_ROBO < 1 || $CURR_ROBO > 3)); then
			echo "Enter the robots that are active (out of [1, 2, 3])";
			exit 1;
		fi
done
echo "fawkes/bbsync/peers:" > $CFG_DIR/startup_generated.yaml

for CURR_ROBO in $ALL_ROBOTS
 do
	IS_INACTIVE=
	for ACTIVE_ROBO in $ACTIVE_ROBOTS
	do
		if [[ "$ACTIVE_ROBO" == "$CURR_ROBO" ]]; then
			IS_INACTIVE=1
			break;
		fi
	done
	if [[ -z "$IS_INACTIVE" ]]; then
		echo "  robot$CURR_ROBO/active: false" >> $CFG_DIR/startup_generated.yaml
	fi
done
for ACTIVE_ROBO in $ACTIVE_ROBOTS
	do
		echo "  robot$ACTIVE_ROBO/active: true" >> $CFG_DIR/startup_generated.yaml
	done

$FAWKES_DIR/bin/fawkes m-central-clips-exec