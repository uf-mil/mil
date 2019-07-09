Sub8 Controller
===============

This package contains various controllers and solvers for small-horizon motion control of the Subjugator vehicle.

The pd_controller here was written in early October, and is nothing more than a simple demo. It only works in simulation, as it subscribes directly to the truth/odom topic published by the sim.

# Running

    rosrun sub8_controller pd_controller
    rosrun rqt_reconfigure rqt_reconfigure


# TODO
* RHC
* LQR

# Tests

* I explicitly did not write gtest unit-tests for this package, because it is redundant with the integration tests, and will never run on the real sub


# Ideas for CPY
- .cpy is a Python tool for C++ code generation
- Automatically fill header files