PID Controller
--------------

SubjuGator 8 uses 6 PID controllers for its 6 degrees of freedom (x, y, z, roll, pitch, yaw). The gain for these PID controllers can be adjusted in ``adaptive_controller.yaml`` found in the ``subjugator_launch/config`` directory. Since the weight of the sub shifts with each component modified, the PID controller values have to be adjusted from time to time. There are two approaches to adjust PID values in the water:

1. Have someone with experience in tuning PID controllers swim with the sub and use the sub's response to movement commands to adjust the gains.
2. Eliminate the error in each axis by adjusting the gains and evaluating the RMS error in each axis after sending the same movement commands.

PID Controller Tuning Tips
~~~~~~~~~~~~~~~~~~~~~~~~~~

* The learning gains (``ki`` and ``kd``) provide very little input to the wrenches applied to the sub, so it is better to treat it as a PD controller to start with. You can disable the learning gains using ``use_learned: false``.
* While you are tuning the PID values, keep a history of the values you have tried for further analysis during and after testing. You can use ``scp`` for this, but it may just be easier to take a screenshot or a photo. You can also take videos of the sub from the side of the pool to reference later.

PID Debugging Data Collection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

    $ roslaunch subjugator_launch bag_debugging_controller.launch prefix_name:="my_prefix"

This launch file records a bag file containing adaptive controller ``pose_error``, ``twist_error``, ``adaptation``, ``dist``, and ``drag`` data along with ``wrench``, ``trajectory``, ``odom``, ``imu/data_raw``, and ``dvl/range`` data. This data is useful in determining if the PID values are getting better or worse using simulations. The resulting bag file will be located in ``gnc/subjugator_controller/debug_bags/``. This launch file was written by Andres Pulido.

.. warning::

    This launch file needs the sub launch file (``sub8.launch``) running in order to function properly.
