# Testing log
Below are notes recorded during NaviGator testing days at Lake Wauburg. They are used as a reference for resolving issues and ideas discovered during testing

## February 4, 2018
* Velodyne was not connected automatically on startup.
  * A manual connection worked, but we need to remedy auto-connect problem.
* In the beginning, we were receiving numerous network kills.
  * Fixed by increasing network timing to 1 second.
* Battery monitor did not launch automatically with NaviGator.
  * Manual launch was necessary, need to find problem with automatic launch.
* Bounds of “safe area” on rviz did not get the correct enu frame coordinates.
  * Needed to rerun.
* As NaviGator pitches in the waves, LIDAR changes.
* Bounds of “safe area” can only be edited by the slider.
  * Text input of bounds does not work on rviz, needs to be fixed.
* From the numerous instances of data collection, NaviGator ran out of disk space.
  * Ran out of disk space leads to a GPS crash, which leads to losing control of NaviGator, which is not good at all.
  * Suggested remedy is an external hard drive with a larger capacity, as losing control is not something that should happen.