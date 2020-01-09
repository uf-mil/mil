# VRX
This page provides general info about developing for [VRX](https://bitbucket.org/osrf/vrx/src), a simulated competition based on the real-life RobotX challenge.
MIL participates both in VRX and RobotX. Most code for this is hosted under NaviGator/.

*NOTE: Please go through the [development guide](/docs/development/development_guide) before going through this tutorial. We will assume you are currently in the development container running a tmux session.*

## Verifying that the VRX Environment is Functional / Playing Around in VRX

### Launch VRX
In one panel, `roslaunch navigator_launch vrx.launch --screen` This will launch the NaviGator Code and the VRX code in the same container.

### Run RVIZ
You can visualize things by running `vrxviz`

### Give a move command
Give NaviGator a move command with `nmove forward 5m`, etc

### See the current odometry
Try streaming the content of a rostopic, `rostopic echo /odom`


## Quick testing
The official VRX system runs our code and the simulator in separated, isolated containers to prevent cheating.
However, for normal development in can be helpful to simply run both in the same container.
To do this, run

`roslaunch navigator_launch vrx.launch run_task:=True world:=<worldfile> --screen`

Where `<worldfile>` is the name of a world under [vrx_gazebo/worlds](https://bitbucket.org/osrf/vrx/src/default/vrx_gazebo/worlds/).

For example, run `roslaunch navigator_launch vrx.launch run_task:=True world:=stationkeeping_task --screen` to test
out the station keeping scenario.

If you wish to see a world with multiple challenges at once (e.g. for testing lidar), try `example_course`

The `run_task:=True` flag tells the mission server to immediately run the VRX mission. Set this to `False` if you wish to manually start the mission.

## Preparing submission
The recommended workflow for submitting our code to vrx is:

1. Run the dev container:
	`./scripts/run_development_container`
2. Branch from the repo and edit whatever files you want to change with what ever text editor you want. Any changes you make to the files from your host machine will immediately show up in the container and when you re-run(for python changes)/ compile(for cpp changes) these changes will immediately take effect. Do the bulk of your development in this stage running mil and vrx code in the same container for fast turn around time in testing.

3. Once you are satisfied with running your code in the development container, you can move on to running your changes against the VRX server.
	
	This is how we will be evaluated by OSRF.

4. Make a Trial Container:

	Commit and push your changes to a git hub repository on a branch. ie: `my_branch` at `https://github.com/ME/mil.git`

	Navigate to the root of the repo.

	* If you pushed to a fork of the uf-mil github: *

	run `./scripts/build_vrx_trial_container my_branch https://github.com/ME/mil.git`
	
	* If you pushed to the uf-mil github: *
	
	run `./scripts/build_vrx_trial_container my_branch`

	*NOTE: Make sure when building your trial container, it the code actually compiles.*
	
5. Run your container with a terminal for sanity check:
	
	Make sure your container actually does what you want when it starts up.
	
	run `./scripts/run_vrx_trial_container my_branch`

6. Now you should have a trial container!

	Follow the instructions at `https://bitbucket.org/osrf/vrx-docker/src/default/` to run this container against the vrx server.

## How to Download and Replay logs from Phase 3 of VRX (2019)

### To Download the Logs
`./NaviGator/simulation/VRX/vrx_logs/2019/download_logs.bash`
### To Play a Log
`./NaviGator/simulation/VRX/vrx_logs/play_log.bash <year> <task> <run>`

i.e. to play the 2019 docking task run 0:

`./NaviGator/simulation/VRX/vrx_logs/play_log.bash 2019 dock 0`

### To Download the Videos:
  `./NaviGator/simulation/VRX/vrx_logs/2019/download_videos.bash`

### To Play a Video:
  `mplayer NaviGator/simulation/VRX/vrx_logs/2019/vrx_2019_videos/<task><run>.mp4`

i.e. to play the 2019 docking task run 1:

  `mplayer NaviGator/simulation/VRX/vrx_logs/2019/vrx_2019_videos/dock1.mp4`

*NOTE: Not all tasks and runs have a video. Navigate to 
`$MIL_REPO/NaviGator/simulation/VRX/vrx_logs/2019/vrx_2019_videos`
to see the avalible videos.*
