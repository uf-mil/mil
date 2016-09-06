Model-Reference Adaptive Controller

################################################# BLACK BOX DESCRIPTION

Inputs:
    - desired pose waypoint [x, y, yaw] in world frame,
      i.e. an end goal of "GO HERE AND STAY"
    - current state (odometry) in world frame

Outputs:
    - the body frame wrench that should then be mapped to the thrusters

################################################# DETAILED DESCRIPTION

This controller implements typical PD feedback, but also adds on
a feedforward term to improve tracking.

The feedforward term makes use of an estimate of the boat's physical
parameters; specifically the effects of drag.

Instead of doing a bunch of experiments to find the true values of
these parameters ahead of time, this controller performs realtime
adaptation to determine them. The method it uses is tracking-error
gradient descent. To learn more, see <https://en.wikipedia.org/wiki/Adaptive_control>.

Additionally, this controller uses the "model-reference architecture."
<https://upload.wikimedia.org/wikipedia/commons/thumb/1/14/MRAC.svg/585px-MRAC.svg.png>
This essentially means that the trajectory generator is built into it.

The model reference used here is a boat with the same inertia and thrusters
as our actual boat, but drag that is such that the terminal velocities
achieved are those specified by self.vel_max_body. This vitual boat
moves with ease (no disturbances) directly to the desired waypoint goal. The
controller then tries to make the real boat track this "prescribed ideal" motion.

The way it was implemented here, you may only set a positional waypoint that
the controller will then plan out how to get to completely on its own. It always
intends to come to rest / station-hold at the waypoint you give it.

Until the distance to the x-y waypoint is less than self.heading_threshold,
the boat will try to point towards the x-y waypoint instead of pointing
in the direction specified by the desired yaw. "Smartyaw." It's, alright...
Set self.heading_threshold really high if you don't want it to smartyaw.

################################################# HOW TO USE

All tunable quantities are hard-coded in the __init__.
In addition to typical gains, you also have the ability to set a
"v_max_body", which is the maximum velocity in the body frame (i.e.
forwards, strafing, yawing) that the reference model will move with.

Use set_waypoint to give the controller A NEW desired waypoint. It
will overwrite the current desired pose with it, and reset the reference
model (the trajectory generator) to the boat's current state.

Use get_command to receive the necessary wrench for this instant.
That is, get_command publishes a ROS wrench message, and is fed
the current odometry and timestamp.

The controller subscribes to the topic /mrac_learn (ROS std bool
message) to either start/continue learning (True) or hault
learning (False). It initializes to False.

################################################# INTERNAL SEMANTICS

*_des: "desired", refers to the end goal, the "waypoint" we are trying to hold at.

*_ref: "reference", it is basically the generated trajectory, the "bread crumb",
       the instantaneous desired state on the way to the end goal waypoint.

*_body: "body frame", unless it is labelled with _body, it is in world-frame.

p: position
v: velocity
q: quaternion orientation
w: angular velocity
a: acceleration
aa: angular acceleration

################################################# OTHER

Author: Jason Nezvadovitz

