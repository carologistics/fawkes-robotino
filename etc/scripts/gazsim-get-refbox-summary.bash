#!/bin/bash
# Outputs the last refbox game summary of the given file


usage()
{
cat << EOF
Usage: $0 file

Outputs the last refbox game summary of the given file
EOF
}

if [[ -z $1 ]]
then
     usage: 
     exit 1
fi

SUMMERY_START=$(grep --line-number 'Awarded Points -- CYAN' $1 | tail -n 1 | sed s/:.*//)
TOTAL_LINES=$(wc -l refbox.log | sed s/.refbox.log//)
let "NUM_LINES_TO_USE = TOTAL_LINES - SUMMERY_START + 1"
SUMMERY=$(tail -n $NUM_LINES_TO_USE $1 | sed s/.*C://)
echo "$SUMMERY"
