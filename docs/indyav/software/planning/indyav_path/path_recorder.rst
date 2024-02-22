Path Recorder
-------------
This is a Utility we use to save a human generated path in the global frame: ``/ecef``.

Indyav Path Recorder inherits from the ``mil_tools TopicRecorder``. We override the CallBack function to record odometry messages at a fixed rate.

Basic Usage Example
^^^^^^^^^^^^^^^^^^^

- Launch `SubjuGator <../../../../subjugator/index.html>`_ in gazebo, clear the kill, and bring up subviz.

- In a new panel, launch the recorder on odom topic:
  ``roslaunch indyav_launch path_recorder.launch record_topic:=/odom``

  *NOTE: This will start to spew the warning :* ``path_recorder frame id is not in ECEF`` *. you can safely ignore this warning for this example.*

- In a new panel, start recording:
  ``service call /path/path_recorder/enable "data: true"``

- In a new panel, command a move forward:
  ``submove f 5``

- Watch the sub move forward 5m in the rviz window

- Stop recording in the same panel as you enabled (doesn't actually matter where this command is issued):
  ``service call /path/path_recorder/enable "data: false"``

- If you like, you can now kill the path recorder by going to the terminal where it was launch and pressing:
  ``Ctrl + c``

- Play back the recording as a normal rosbag on a different topic and visualize it:
  ``rosbag play ~/test.bag  /odom:=/odom2 -l``

- Visualize this new topic in rviz by clicking `Add` and then `/odom2 -> Odometry`

- Watch the red arrow advance from where the sub started to where the sub stopped updating approximately 10 times a second.

Source Files
^^^^^^^^^^^^
indyav_path PathRecorder:
    header:
        ``rosed indyav_path path_recorder.hpp``
    source:
        ``rosed indyav_path path_recorder.cpp``

mil_tools TopicRecorder:
    header:
        ``rosed mil_tools topic_recorder.hpp``
    source:
        ``rosed mil_tools topic_recorder.cpp``

launch file:
    ``rosed indyav_launch path_recorder.launch``
