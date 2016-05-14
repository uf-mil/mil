# Gazeb

A brief tutorial on how to gazeb.

> If you wish to simulate the sub from scratch, you must first invent the universe. <br>
> &nbsp;&nbsp;&nbsp;-- Carl Sagan

### To run the sim
To launch with the gui:
`roslaunch sub8_gazebo duck.launch`

To launch without the gui: `roslaunch sub8_gazebo duck.launch gui:=false`

To launch without the cameras: `roslaunch sub8_gazebo duck.launch cameras:=false`

And these can be combined if you'd like.

**NOTE:** You can run the sub with or without a gui. Disabling one or both of these is intended for use when testing controllers or other things that only rely on the data and not the actual visuals. Also it is intended for people who have shitty computers that would benefit from not having to run the gazebo window and the cameras (which saves a nontrival amount of CPU time).