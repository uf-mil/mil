# Testing log
Below are notes recorded during NaviGator testing days at Lake Wauburg. They are used as a reference for resolving issues and ideas discovered during testing

## February 4, 2018
* Velodyne was not connected automatically on startup.
  * Worked after reconnecting coupler outside of electronics box, either connection is bad or some network issue
* In the beginning, we were receiving numerous network kills.
  * Potentially caused by maxing out ubiquity bandwidth from uncompressed camera feed
  * Fixed by increasing network timing to 1 second.
* Battery monitor did not launch automatically with NaviGator.
  * Manual launch was necessary, need to add to launc file
* Bounds of “safe area” on rviz did not get the correct enu frame coordinates.
  * Needed to rerun.
  * likely because bounds server runs before odom published
* As NaviGator pitches in the waves, LIDAR changes.
  * might not be a problem, but should be sure TF is accurate
* Bounds of “safe area” can only be edited by the slider.
  * Text input of bounds does not work on rqt (bug in RQT, it's inconsistent)
* From the numerous instances of data collection, NaviGator ran out of disk space.
  * Ran out of disk space leads to a GPS crash, which leads to losing control of NaviGator, which is not good at all.
  * Suggested remedy is an external hard drive with a larger capacity, as losing control is not something that should happen.