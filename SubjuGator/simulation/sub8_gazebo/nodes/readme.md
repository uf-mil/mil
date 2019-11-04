Gazebo Job Runner
=================

## Premise
The idea is that anyone will be able to make a mission script that will do some task. With `job_runner.py` running, the mission script will be executed in a
loop and should be set up to report successes or failures of the mission. If a mission fails bag data from around that failure will be saved.

## To Use with Your Script
There are three main components to include in your script in order to integrate with the Job Runner.

1. Your script should advertise a `RunJob` service on `/gazebo/job_runner/mission_start`. The job runner will send a start command once this is found.
2. Your script needs to return wether it was successful or not using that service call that the job runner sent out. Your script also should be able to catch its own errors and report them as well.
3. When you recieve the start command, your script should be able to set the subs position and any field components you want if they are not already there. The job runner will reset the gazebo simulation
after every run.
