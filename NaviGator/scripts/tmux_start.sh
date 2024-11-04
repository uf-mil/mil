#!/bin/bash
if tmux has-session -t auto; then
	tmux a -t auto
else
	tmux new-session -d -s auto
	tmux send-keys -t auto:0.0 'roslaunch navigator_launch master.launch --screen' Enter
	tmux split-window -h -t auto
	tmux split-window -v -t auto
	tmux new-window -t auto
	tmux split-window -h -t auto:1
	tmux split-window -v -t auto:1
	tmux split-window -v -t auto:1.0
	sleep 1.5
	tmux send-keys 'amonitor kill' Enter
	tmux split-window -h
	tmux send-keys 'amonitor hw-kill' Enter
	tmux select-pane -t auto:1.0
	tmux a -t auto
fi
