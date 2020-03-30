Path Player
-----------

This is a utility that we use to play back a path that we recorded from earlier, so that we can test systems not related to path planning.

Path Player is just an instance of ``mil_tools TopicRecorder``. Its current behavior is that it will publish all messages in a bag file(our recorded path) in sequential order with correct timing between them. This utility is designed to be inherited from and the ``Play`` function overriden.

Basic Usage Example
^^^^^^^^^^^^^^^^^^^

- Complete the `Path Recorder Basic Usage Example First <path_recorder.html>`_.

- Launch `SubjuGator <../../../../subjugator/index.html>`_ in gazebo and clear the kill.

- In a new panel, play the previously recorded bag file.
``roslaunch indyav_launch path_player.launch``

- Visualize in Rviz by adding ``/path``.

- Witness the new red arrow trace the path of SubjuGator from earlier.


Source Files
^^^^^^^^^^^^

indyav_path PathPlayer:
    source:
        ``rosed indyav_path path_player.cpp``

mil_tools TopicPlayer:
    header:
        ``rosed mil_tools topic_player.hpp``
    source:
        ``rosed mil_tools topic_player.cpp``
launch file:
    ``rosed indyav_launch path_player.launch``
