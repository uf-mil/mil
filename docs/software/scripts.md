# Scripts

At MIL, we use a lot of scripts to help us complete tasks quickly! Check out some things you can do with them:

## Occupancy Grid

**Draw an occupancy grid:**

    $ roscd mil_tools && ./mil_tools/nodes/ogrid_draw.py

**Maintains a bag file with multiple topics:**

    $ roscd mil_tools && ./mil_tools/nodes/online_bagger.py

## Keyboard Control
To control the robot using the keyboard, use the ``KeyboardClient`` and ``KeyboardServer``
classes inside of the ``navigator_keyboard_control`` package.

The two nodes within the package can be run through ``rosrun``.

## Alarms
**Clear an alarm:**

    $ roscd ros_alarms && rosrun nodes/clear.py

**Monitor an alarm:**

    $ roscd ros_alarms && rosrun nodes/monitor.py

**Raise an alarm:**

    $ roscd ros_alarms && rosrun nodes/raise.py

**Reports the status of an alarm:**

    $ roscd ros_alarms && rosrun nodes/report.py
