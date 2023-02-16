#!/bin/bash

function log_to_facts() {
        if [ -z ${2+x} ]; then
                sort -k 6 $1  | uniq --unique -c -f 6 | sed '/==>/!d' | sort -n
        else
            	head -n $2 $1 | sort -k 6 | uniq --unique -c -f 6 | sed '/==>/!d'
        fi
}

echo "Generating filtered log-files..."
grep -e "GOAL-.*mode" debug11_latest.log | sed 's/^.*goal (id //g' | sort -u > debug11_goals.log
grep "(state " debug11_latest.log > debug11_states.log
log_to_facts debug11_latest.log > debug11_facts.log
echo "Finished generating files!"
