#!/bin/bash
set -e

function usage() {
cat << EOF
USAGE:
  ./rcll_netemulation setup <interface>
  ./rcll_netemulation clear <interface>
  ./rcll_netemulation show  <interface>
EOF
}

function setup_rules() {

   # The classless netem qdisc does not allow use of filters.
   # In order to apply filters to netem qdisc it has to be encapsulated into a classfull qdisc like prio

   # prio creates 3 classes:
   # x:0| qdisc itself
   # x:1| class 1 highest priority
   # x:2| class 2
   # x:3| class 3 lowest priority

   # netem minimum limit calculation:
   # <bandwith> / <MTU Size> * delay * 1.5
   # 1 Gbps / 1500 bytes MTU * 100 ms * 1.5 = 12500.

    tc qdisc add dev $1 root handle 1: prio
    tc qdisc add dev $1 parent 1:1 handle 10:  netem limit 100000 delay 2000ms 200ms 25% loss 20% 25%

   # Protocol ID's
   # icmp 1
   # tcp  6
   # udp  17

   # flowid redirects packet to the corresponding class

   # Matching Rules:
   # Each of these filters are traversed with Logical OR
   # match each icmp package that is encapsulated by an ip packet
    tc filter add dev $1 protocol ip parent 1: prio 1 u32 match ip protocol 1 0xff flowid 1:1

   # match all udp packages with destination port 5001
   # in order to filter out udp packages use "ip dport" instead of "udp dst" due to "implicit" nexthdr not forwarded into the queue
   # 0xffff is the mask that has to be applied in order to match the udp and tcp headers
    tc filter add dev $1 protocol ip parent 1: prio 1 u32 match ip protocol 17 0xff match ip dport 5001 0xffff flowid 1:1
}

function clear_rules() {
    sudo tc qdisc del dev $1 root
}

function show_rules() {
    echo 'QDiscs:'
    tc qdisc show dev $1
    echo 'Classes:'
    tc class show dev $1
    echo 'Filters:'
    tc filter show dev $1
}

function main() {
    case $1 in
        show)
            show_rules $2
        ;;
        setup)
            setup_rules $2 $3
        ;;
        clear)
            clear_rules $2
        ;;
        *)
            usage
    esac
}

main $@
