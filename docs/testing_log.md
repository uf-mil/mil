# Testing Log
This page tracks notes from testing days.

## January 26, 2020 - Sub8 pool test
Launch checklist:
  - Update git to correct branch (latest github master or whatever you're testing)
  - Compiled with `cm`
  - Run `roslaunch sub8_launch sub8.launch`
  - Bring up amonitor kill in another tmux pane, be prepared to kill at any time
  - Look for any errors, investigate
  - Put sub in water
  - have operator move sub around, verify odom visually in subviz
  - 

Notes:
  - Issues with socat connection to DVL, fixed itself after two restarts
  - Buoytancy had us pitching up, could not control for it with gains in repo or 2018 competition gains
  - With competition gains, sub was applying hard wrench in wrong direction
    - competion gains had learning on, turned off
  - Retuned and get fairly stable
  - Think we should add learning both with respect to the world and the body
    - Or 
