#!/bin/bash

# ***************************************************************************
# *  Script based on tc to emulate network problems
# *
# *  Created:  Oct 13 15:31:57 2018
# *  Copyright  2018  Christoph Gollok [christoph.gollok@alumni.fh-aachen.de]
# *
# ****************************************************************************
#
# *  This program is free software; you can redistribute it and/or modify
# *  it under the terms of the GNU General Public License as published by
# *  the Free Software Foundation; either version 2 of the License, or
# *  (at your option) any later version.
# *
# *  This program is distributed in the hope that it will be useful,
# *  but WITHOUT ANY WARRANTY; without even the implied warranty of
# *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# *  GNU Library General Public License for more details.
# *
# *  Read the full text in the LICENSE.GPL file in the doc directory.
# *

set -e
# TODO GET Ports from config file
PORTS=(27017 27021 27022 27023 27031 27032 27033)
DEBUG=false

function usage() {
cat << EOF
Allows to modify network Traffic. Based on 'tc' tool

usage:

  $0 setup <network interface> OPTIONS
  $0 clear <network interface>
  $0 show  <network interface>

OPTIONS:
   -h|--help
   -d|--delay <delay in ms> <random uniform distribution in ms> <correlation in %>
   -c|--corruption <value in %>
   --duplicate <value in %>
   -r|--rate limit bandwith <value in kbit or Mbit>
   -D|--Debug        Apply rules also to icmp. Use ping to test setup
e.g.

$0 setup lo -c 60% -d 800ms 50ms 25% --duplicate 10% -r 2Mbit -D

EOF
}

function setup_rules() {
set +e
clear_rules $DEVICE
set -e
# The classless netem qdisc does not allow use of filters.
# In order to apply filters to netem qdisc it has to be encapsulated into a classfull qdisc like prio

# prio creates 3 classes:
# x:0| qdisc itself
# x:1| class 1 highest priority
# x:2| class 2
# x:3| class 3 lowest priority
    echo "setup rules for device $DEVICE"
    tc qdisc add dev $DEVICE root handle 1: prio

# netem minimum limit buffer calculation:
# <bandwith> / <MTU Size> * delay * 1.5
# 1 Gbps / 1500 bytes MTU * 100 ms * 1.5 = 12500.


filter="tc qdisc add dev $DEVICE parent 1:1 handle 10:  netem limit 100000 "

if [ ! -z $DELAY ];
then
	echo "delay: $DELAY"
       	echo "delay distribution: $DELAY_DISTRIBUTION"
	echo "delay correlation: $DELAY_CORRELATION"
	filter+="delay $DELAY $DELAY_DISTRIBUTION $DELAY_CORRELATION "
fi

if [ ! -z $LOSS ];
then
    echo "packet loss: $LOSS"
    echo "loss correlation: $LOSS_CORRELATION"
    filter+="loss $LOSS $LOSS_CORRELATION "
fi

if [ ! -z $CORRUPTION ];
then
    echo "packet corruption: $CORRUPTION"
    filter+="corrupt $CORRUPTION "
fi

if [ ! -z $RATE ];
then
    echo "bandwith: $RATE"
    filter+="rate $RATE "
fi

if [ ! -z $DUPLICATE ];
then
    echo "duplicates: $DUPLICATE"
    filter+="duplicate $DUPLICATE"
fi

eval $filter

# Matching Rules:
# Each of these filters are traversed with Logical OR
# match each icmp package that is encapsulated by an ip packet
if [ "$DEBUG" = true ] ; then
	echo "Debug enabled. Apply filter on icmp protocol. use ping to test your setup"
   	tc filter add dev $DEVICE protocol ip parent 1: prio 1 u32 match ip protocol 1 0xff flowid 1:1
fi
# in order to filter out udp packages use "ip dport" instead of "udp dst" due to "implicit" nexthdr not forwarded into the queue
# 0xffff is the mask that has to be applied in order to match the udp and tcp headers

# match all tcp packages that go to tcp dport 27017, 27021 and 27031
echo setup rules for ports:
for i in "${PORTS[@]}"
do
	echo $i
        tc filter add dev $DEVICE protocol ip parent 1: prio 1 u32 match ip protocol 6 0xff match ip dport $i 0xffff flowid 1:1
done

}

function clear_rules() {
echo "clear rules on dev $1"
tc qdisc del dev $1 root
}

function show_rules() {
    echo 'QDiscs:'
    tc -p qdisc show dev $1
    echo 'Classes:'
    tc -p class show dev $1
    echo 'Filters:'
    tc -p filter show dev $1
}

function main() {
    case $1 in
        show)
            show_rules $2
        ;;
        setup)
	    DEVICE=$2
	    while [[ $# -gt 2 ]]
	    do
	     	arg="$3"
	    	case $arg in
    		    -d|--delay)
		    DELAY="$4"
		    DELAY_DISTRIBUTION="$5"
		    DELAY_CORRELATION="$6"
		    shift # argument
		    shift # value
		    shift # value
		    shift # value
		    ;;
    		    -l|--loss)
		    LOSS="$4"
		    LOSS_CORRELATION="$5"
		    shift #argument
		    shift #value
		    shift #value
		    ;;
		    -c|--corruption)
		    CORRUPTION=$4
		    shift #argument
		    shift #value
		    ;;
	            --duplicate)
		    DUPLICATE=$4
		    shift # argument
		    shift # value
		    ;;
	            -r|--rate)
		    RATE=$4
		    shift # argument
		    shift # value
		    ;;
                    -D|--Debug)
		    DEBUG=true
		    shift #argument
		    shift #value
		    ;;
		    -h|--help)
		    usage
		esac
    	    done
            setup_rules
        ;;
        clear)
            clear_rules $2
        ;;
        *)
            usage
    esac
}

main $@
