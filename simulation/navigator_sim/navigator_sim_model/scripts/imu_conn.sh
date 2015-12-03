#!/bin/sh
exec socat -d -d pipe:/tmp/imu,wronly=1 tcp:127.0.0.1:10025
