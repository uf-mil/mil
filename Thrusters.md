Thrusters
=========

We have 8 VideoRay M5 Thrusters on Subjugator 8. These are the primary mode of actuation for vehicle motion, other than throwing.

# Thrust Allocation
We use a simple sequential least squares solver, provided by Numpy, to compute a solution. We will eventually move to a faster QP solver.

## Terms

* **thrust vector** - A (8 element) vector, **u**, where **u**[n] is the thrust from thruster_n
* **control input matrix** - A matrix, **B**, where B.dot(u) yields the *wrench* applied to the sub
* **wrench** - a vector, **w**, a 6 element vector: [XYZ Forces, XYZ torques]

## Method
**Mundane setup:** 

We query the thrusters for their bounds, and then set up a control-input matrix, B, using our knowledge of the vehicle's thruster configuration. This knowledge is recovered from a parameter stored on '/busses'. This parameter contains the locations and and directions of the thrusters relative to the center-of-mass-centered, FLU coordinate system of the sub. 

**Computation setup:** 

Each column of B represents the contribution of a single thruster to the sub's net wrench. This looks like [direction_vector, cross(position_vector, direction_vector)], transposed.

B is a 6x8 matrix, so that B * u yields a wrench which is a 6x1 column vector.

**Solving:** 

We compute a solution by minimizing k<sub>1</sub>\*norm(B\*u - w<sub>desired</sub>) + k<sub>2</sub>\*norm(u). 

This amounts to a multi-criterion optimization problem, and by adjusting the scaling of error cost (k<sub>1</sub>) and norm(u) cost (k<sub>2</sub>), we select a pareto *optimal* solution. So long as we both k<sub>1</sub> and k<sub>2</sub> are *positive*, there will always be a unique *optimum* for our vehicle.

# Notes

- The individual thrusters within the driver should not have or need knowledge of their location on the physical vehicle
- Tracking positions through TF is a conversation to have - I am not convinced that it is worth extra  headaches spawned by publishing the static transforms. Instead, the positions and directions are held in a rosparam whose format was defined by Forrest.

# Hardware Documentation
[Faults](http://download.videoray.com/documentation/m5_thruster/html/configuration_mode.html)
[User's Manual](http://download.videoray.com/documentation/m5_thruster/html/)

# TODO

* [ ] Implement numerical thrust allocation
    * [x] Determine thruster layout configuration file format
    * [x] Construct control-input matrix from configuration
    * [ ] Handle thruster-out events
* [x] Implement thruster communication protocol
    * [x] Implement simulated thruster output
        * [ ] Expose services for simulating thruster-out events
* [ ] Implement ROS thruster interface
    * [x] Handle thruster Newtons -> Thruster input calibration
    * [x] Implement srv/msg types
        * Thrusters should communicate their capabilities
    * [x] Implement thruster info publisher
    * [x] Publish thruster status messages
    * [ ] Implement alarms
        * [ ] Implement thruster-out handling
        * [ ] Add temp/voltage alarms
    * [x] Shut down after a timeout
* [ ] Testing
    * [x] Add nosetests
    * [x] Add rostest integration tests
* Stretch Goals
    * [ ] Poll the thrusters without issuing a command

# References

[1] Goldstein, Andy; [VideoRay m5 git repository](https://github.com/videoray/Thruster)

[2] Voight, Forrest; [Old Implementation](https://github.com/uf-mil/software-common/blob/master/videoray_m5_thruster_driver/scripts/videoray_m5_thruster_driver)