#!/bin/bash
echo "Generating filtered log-files..."
grep "action-name visit" debug11_latest.log > debug11_visit.log
grep "(state " debug11_latest.log > debug11_states.log
grep "(state RUNNING)" debug11_states.log > debug11_running.log
grep "(state FINAL)" debug11_visit.log | sed 's/^.* (plan-action //g' | sed 's/(start-time.*$//g' | sort -u > debug11_final.log
echo "Finished generating files!"
