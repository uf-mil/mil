#!/bin/bash

sdgps generate-fake-sonar-solution --rate 1 --position '[-30, 10, -2]' --circle '{"radius":3,"axis":[0,0,1],"period":30}' ! sonar-simulate --role rover --transmit-pattern robosub-2019 --base-hydrophone-arrangement robosub-2019 --rover-hydrophone-arrangement uf-mil-sub7 --cn0 80 --multipath extreme ! filter-sonar-raw-samples --highpass 15e3 --lowpass 45e3 --notch 40e3 ! extract-robosub-pings ! robosub-ping-solver ! listen-robosub-ping-solution-tcp 2007
