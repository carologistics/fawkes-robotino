#!/bin/bash
# Outputs the last refbox game summery of the given file


usage()
{
cat << EOF
Outputs the last refbox game summery of the given file

Usage: gazsim-get-refbox-summery.bash file
EOF
}

if [[ -z $1 ]]
then
     usage: 
     exit 1
fi

SUMMERY_START=$(grep --line-number 'Awarded Points' $1 | tail -n 1 | sed s/:.*//)
TOTAL_LINES=$(wc -l refbox.log | sed s/.refbox.log//)
let "NUM_LINES_TO_USE = TOTAL_LINES - SUMMERY_START + 1"
SUMMERY=$(tail -n $NUM_LINES_TO_USE $1 | sed s/.*C://)
echo "$SUMMERY"
