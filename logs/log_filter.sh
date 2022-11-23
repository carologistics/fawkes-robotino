#!/bin/bash
echo "Generating filtered log-files..."
grep "action-name visit" debug11_latest.log > debug11_visit.log
grep "(state " debug11_latest.log > debug11_states.log
grep "(state RUNNING)" debug11_states.log > debug11_running.log
echo "Finished generating files!"
