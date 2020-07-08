Passive sonar
=============


How to Use
----------

Launch
******
``roslaunch sub8_launch passive_sonar.launch``

Debugging
*********

To Play Back a File
^^^^^^^^^^^^^^^^^^^

* Download `oof.bin <http://sylphase.com/files/oof.bin>`_
* ``roslaunch sub8_launch passive_sonar.launch environment:=file``

To step through a file
______________________
* ``rosrun mil_tools stream_tcp_dump ~/Downloads/oof.bin --batch_size=960000 --advance_on_arrow=True``

To Play back a file at real speed
_________________________________
* ``rosrun mil_tools stream_tcp_dump ~/Downloads/oof.bin --batch_size=960000 --rate=10``


Visualize Data in the Pipeline
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
This pipline makes use of the ``mil_common/utils/mil_tools/mil_ros_tools/plotter.py`` utility to plot batches of data over on ROS Topics as CV2 images at most points in the processiong pipeline.

To view all points of the triggering pipline
____________________________________________
(raw, max convolution, gradient) that caused the most recent trigger (only hydrophone 0), look at ``/hydrophones/triggering/trigger_debug``

To view raw samples from all hydrophones around the triggering time
____________________________________________________________________
look at ``/hydrophones/triggering/sample_at_trigger_debug``

To view the frequency response of the bandpass filter
_____________________________________________________
open ``/hydrophones/triggering/filter_debug`` in rviz. Then, in a new pannel ``rosservice call /hydrophones/triggering/filter_debug_trigger "{}"`` Go look at the rviz and see the frequency response of the filter cropped for our general frequency range.

To view the signals sent to the ping locator node
_________________________________________________
(and vertical lines at the trigger times) view ``/hydrophones/ping_locator/samples_debug``
*NOTE:The Vertical lines are where triggering is detected on each hydrophone channel.*

To view the cross correlations between the hydrophones
______________________________________________________
(and therefore the time delays) look at ``/hydrophones/ping_locator/cross_correlation_debug``
*NOTE:The Vertical lines are the max of the cross correleation and therfore the negative of the time delays.*


Configuration
*************
All parameters that are expected to be changed in tuining are ROS Params initialized in ``passive_sonar.yaml``

To make a custom configation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* ``roscd sub8_launch``

* ``cp config/passive_sonar.yaml config/my_passive_sonar.yaml``

* ``rosed my_passive_sonar.yaml``

* change whatever parameters you like

* ``roslaunch sub8_launch passive_sonar.launch config_file:=my_passive_sonar.yaml``

To change the configuration at runtime
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* set whatever param you want to change ie:``rosparam set /hydrophones/triggering/target_frequency 25000``

* call the reset service ie: ``rosservice call /hydrophones/triggering/reset "{}"``


Testing ROS bridge
------------------
* Download `example TCP dump, <http://sylphase.com/files/oof.bin>`_
* Run ROS bridge ``rosrun mil_passive_sonar sylphase_sonar_ros_bridge _port:=10001 _ip:=127.0.0.1``
* Either stream a TCP dump or run the driver on a real robot
  * Stream TCP dump ``rosrun mil_tools stream_tcp_dump  ~/Downloads/oof.bin --port=10001 --ip 127.0.0.1``
  * On Sub8 run the actual driver ``(cd ~/.mil/sylphase-sonar/; ./driver/publish 10001)``
* Make sure samples are being published ``rostopic echo /samples``

How Does It Work?
-----------------

.. graphviz:: passive_sonar.dot
