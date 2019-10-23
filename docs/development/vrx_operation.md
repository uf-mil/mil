# Operating VRX
This guide walks you through playing around in VRX

1. Run the development container `./scripts/run_development_container`
1. Start a tmux session from the container (allows you to have multiple terminals within the container) `tmux new`
1. Split the tmux session with `Control+B` then `"`. You can switch between the terminals with `Control+B` then up arrow / down arrow
1. In one panel, [run vrx](/docs/development/vrx)
1. See below sections about other things to do in other panes

## Run RVIZ
You can visualize things by running `vrxviz`

## Give a move command
Give NaviGator a move command with `nmove forward 5m`, etc


