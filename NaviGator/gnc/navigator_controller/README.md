There are currently two controller options.

If you want to switch between them, open gnc.launch and edit the
line defining the pose_controller node according to the comment there.

If you are using MRAC controller, you will notice a class attribute
called use_external_tgen. If you manually set this to True in the source
code, it will subscribe to something that publishes on /trajectory (i.e.
the C3 trajectory generator from the sub, etc...). If you want to play it
safe and use the built in trajectory generator, keep it False and
it will only need to describe to a /waypoint which is a pose representing
the end goal. The move_helper node (built into the NaviGator GUI) uses this.

If you are using the NN_controller, you will need the package in your
catkin_ws as it is an external dependency. The package can be found here
https://github.com/jnez71/adaptive_control

The rest of the documentation for these controllers is found in their class
module source code.

Contact Jason Nezvadovitz if you have any questions.
