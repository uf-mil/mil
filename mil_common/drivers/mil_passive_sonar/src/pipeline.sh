#!/bin/bash

sdgps connect-sonar-raw-tcp --host 192.168.37.51 --port 2006 ! filter-sonar-raw-samples --highpass 15e3 --lowpass 45e3 --notch 40e3 ! extract-robosub-pings ! robosub-ping-solver ! listen-robosub-ping-solution-tcp 2007
