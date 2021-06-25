Control
=======

.. toctree::
    :maxdepth: 1

    IndyAV Joydrive <joydrive/joydrive.rst>

Control Packages
----------------
No software developed yet.

Discussion
-------------
Initial development will start simple.

Pure Pursuit Steering Control
*****************************
Initial implementations for the path tracking controller will be a pure pursuit controller

PID Speed Control
*****************
The speed control will consist of some combination of proportional, integral, and derivative.
The error state will be the delta between a desired and measured forward speed.

Future Work
-----------
* MPC controller will probably be looked into.

