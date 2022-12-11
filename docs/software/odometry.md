# Basic Movement and Odometry

All of our robots use a method of position estimation known as odometry.
Odometry is the process of integrating multiple sources of information to
determine the robot's position relative to where it started. This system allows
a robot to make very precise movements based on its idea of its position.

This technique produces an odometry reading, commonly represented by the
{class}`nav_msgs.msg.Odometry` ROS message type. This message type contains a
pose, the robot's current position and orientation, and a twist, the robot's
current linear and angular velocity.

But, why not a method like GPS? After all, that is how many electronic devices
(for example,a  GPS) get a representation of their position. The two major
flaws with this approach include the fact that GPS is not very accurate, and
the fact that using one source of information is weaker than using multiple
sources of information. Furthermore, GPS can't penetrate water! So, no GPS
capabilities for a robot like SubjuGator. *sad face*

## Covariance

When viewing odometry readings in ROS, you will notice another measurement:
covariance. This is a 36-value covariance matrix, showing the covariance
between 6 unique values: x, y, z, rotation about the X, rotation about the Y,
and rotation about the Z. The covariance is indicated between each of the
values - for example, the first element in the array is the covariance between
x and x, the next element is the covariance between x and y, etc. These
covariance values indicate the relationship between two unique variables.
Positive covariance values indicate that there is a positive linear
correlation, ie, as one of the variables increases, the other is expected to
increase alongside it (vice versa, for a negative covariance value).
Furthermore, the higher the covariance value, the closer the two variables
change together.

But, why are these covariance values here? If you read them yourself, it can
look like random noise. Interestingly, these covariance values hold some
interesting information if you know how to compute statistics on them properly.
One application of these values is error ellipses. These error ellipses
indicate the perceived error in a particular measurement - the bigger the
ellipse, the more error perceived. This concept is similar to confidence
ellipses, which aim to highlight the perceived confidence in a particular
measurement. Be careful to not confuse these two values!

In practice, however, these values are rarely used outside of the Kalman filter
calculations.

## Derivation

How does odometry get derived? As mentioned earlier, multiple source of
information are taken into account. How this is done varies by each robot.

For SubjuGator, these sources get put through an extended Kalman filter, known
as the [`odom_estimator`](/design/odom_estimator/odom_estimator), originally
designed by MIL member Forest Voight. This estimator, written in C++, combines
information from the Doppler Velocity Log (DVL), Inertial Measurement Unit
(IMU), and a depth sensor. The accelerometer, magnetometer, and gyroscope
measurements are used from the IMU. This Kalman filter is quiet powerful,
especially when eliminating errors. Sensors including the inertial measurement
unit (IMU) build up error over time, however, this error is eliminated through
the Kalman filter.

Furthermore, a small amount of redundancy is built into the Kalman filter. The
filter is able to record knowledge about which DVL beam failed; it can use this
information to operate on only one or two DVL beams.

## Using Odometry

With all of the information contained in a single odometry reading, odometry is
quite a powerful message type. Therefore, odometry is used by several systems,
including the transforms system (so other transforms can reference the robot's
updated position), the alarm server (to kill the robot if it moves into a
dangerous position), and the mission system (to move the robot according to a
high-level plan).

Odometry is also useful for debugging the robot. Odometry can be quickly viewed
using the `rostopic echo` command:

```bash
rostopic echo /odom
```

## Basic Movement

Awesome, now that we understand some basics about how odometry is derived, how
can we use this reading to move our robot? Each robot does this differently, as
well. For SubjuGator, this is the job of the C3 Trajectory Generator, a system
to plan out trajectories in a C3 state space (three-dimensional state space).
This system generates the trajectories for SubjuGator; these trajectories are
then sent to the sub's adaptive controller system, which turns the trajectories
into actual movements.

### C3 Trajectory Generator

The C3 trajectory generator, written in C++, is a small ROS package which
implements an [action server](http://wiki.ros.org/actionlib) under the
namespace `moveto` (not to be confused with `move_to`, the namespace used by
NaviGator's action server). This server can be sent goals by various systems,
and the trajectory generator will continuously figure out the most optimal path
to the point until the point has been reached.

### Adaptive Controller

SubjuGator's adaptive controller is a single Python file which listens to the
trajectory messages sent by the trajectory generator.

### Thruster Mapper

Great! We now have a wrench message describing the goal force and torque to be
exerted on the robot! But, we still need to actually power the individual
motors to achieve this goal. This is where the thruster mapper comes in - the
goal of the thruster mapper is to do exactly this.

## `submove` - The End Goal

At the end of all of this, we are able to move the sub along any of its degrees
of freedom using the `submove` command. This interface allows us to change the
pitch, roll, yaw of the robot, alongside its orientation in the x, y, and z
directions. Having a high-level interface like this makes missions easier to
write, and demonstrations cooler!
