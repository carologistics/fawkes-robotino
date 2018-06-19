#!/bin/bash
set -e

function usage() {
cat << EOF
USAGE:
  ./gazsim_rc_network_sim setup <interface>
  ./gazsim_rc_network_sim clear <interface>
  ./gazsim_rc_network_sim show  <interface>
EOF
}

function setup_rules() {
    tc qdisc add dev $1 root handle 1: prio
    tc qdisc add dev $1 parent 1:3 handle 30: netem delay 100ms 
    tc filter add dev $1 protocol ip parent 1:0 u32 match ip sport 5001 0xffff flowid 1:3
    tc filter add dev $1 protocol ip parent 1:0 u32 match ip sport 27031 0xffff flowid 1:3
    tc filter add dev $1 protocol ip parent 1:0 u32 match ip sport 27021 0xffff flowid 1:3
    tc filter add dev $1 protocol ip parent 1:0 u32 match ip sport 27017 0xffff flowid 1:3
}

function clear_rules() {
    sudo tc qdisc del dev $1 root
}

function show_rules() {
    tc qdisc show dev $1
    tc class show dev $1
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
