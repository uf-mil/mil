#!/bin/bash
if tmux has-session -t auto; then
	tmux a -t auto
else
	# First window (panes)
	tmux new-session -d -s auto
	tmux send-keys -t auto:0.0 'roslaunch navigator_launch master.launch --screen' Enter
	tmux split-window -h -t auto
	tmux split-window -v -t auto

	# Second window (alarms, other panes)
	sleep 1.5
	tmux new-window -t auto
	tmux split-window -h -t auto:1
	tmux split-window -v -t auto:1
	tmux split-window -v -t auto:1.0
	tmux send-keys 'amonitor kill' Enter
	tmux split-window -h
	tmux send-keys 'amonitor hw-kill' Enter
	tmux select-pane -t auto:1.0

	# Return to the first window
	tmux select-window -t auto:0
	tmux a -t auto
fi
