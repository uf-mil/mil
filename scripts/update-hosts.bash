#!/bin/bash

HOSTFILE=/etc/hosts
HOSTS=(	"192.168.1.1	mil-plumbusi"
	"192.168.1.2	mil-access-point"
	"192.168.1.3	mil-johnny-five"
	"192.168.1.4	mil-eve"
	"192.168.1.5	mil-sw"
	"192.168.1.10	printer"
	"192.168.1.20	sub-ipmi"
	"192.168.1.21	sub"
	"192.168.1.22	sub-gumstick"
	"192.168.1.23	sub-tx1"
	"192.168.1.24	sub-imaging-sonar"
	"192.168.1.30	jacobtop"
	"192.168.1.31	ralph-mbp"
	"192.168.1.32	MattLinux"
	"192.168.1.33	anthony-pc"
	"192.168.1.34	WLC0002"
	"192.168.1.35	santi-K53SD"
	"192.168.1.36	zach"
	"192.168.1.37	lucas-Yoga-13"
)

if !(cat $HOSTFILE | grep --quiet "# MIL hostnames for Sub8"); then
    sudo sh -c "echo '' >> $HOSTFILE"
    sudo sh -c "echo '# MIL hostnames for Sub8 (this block of hosts should be last in the hosts file)' >> $HOSTFILE"
fi

TMP=$(mktemp /tmp/update-hosts.XXXXXX)

for ((I=0; I < ${#HOSTS[@]}; I++)); do
    if !(cat $HOSTFILE | grep --quiet "${HOSTS[$I]}"); then
        HOST="${HOSTS[$I]}"
        echo "$HOST" >> $TMP
    fi
done

sudo sh -c "cat $TMP >> $HOSTFILE"

rm -rf $TMP
