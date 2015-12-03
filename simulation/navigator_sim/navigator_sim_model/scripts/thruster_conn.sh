#!/bin/sh
exec socat -d -d pty,link=$1,raw,echo=0 tcp:127.0.0.1:$2
