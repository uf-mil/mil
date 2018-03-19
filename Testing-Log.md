# Testing log
Below are notes recorded during NaviGator testing days at Lake Wauburg. They are used as a reference for resolving issues and ideas discovered during testing

## March 18, 2018
* GPS programs were crashing - switched output file to /dev/null, fixed problem
* Can't write bags to external bag drive - bag drive mounted as read-only - source of GPS errors
* Unplugged from one USB port to another, fixed read-only problem. Try to replicate in lab...
* Velodyne could not connect again,  moved to different port on switch and it worked
* Bug in circle option for move command, fix and on lakeday branch
* Every time xbox controller reconnects, it kills
* Kayakers should notify mission control if they're swimming in the water, as it is hard to see them
* Blind move did not work
* Need option for not using rrt, just continuous path

## February 18, 2018
* Had some issues with running both pointgrey cameras at once
* The smallest sized competition (black) buoys become blind to the velodyne at about 4.8m away (measured in 2D from velodyne to buoy)
* Seecam uses a lot of bandwith / storage when bagging because 1920x1080@60fps, could probably get away w/ 15fps
* Need to add a tf for Seecam (can reuse front_right cam for now)
* Battery monitor inaccurate near startup when it has few measurements (some of which may from killed readings), should wait till buffer is full
* Could not run pointgrey driver at some point because of git issues (be sure to update submodules)
* Task runner should ignore a task with syntax error (just warn and proceed without it)
* Should disable perception launch as lots of old broken code and not being used
* Should add mission to circle a point while maintaining CURRENT radius to that point (now just hardcoded 3 meters)
* Visualization was kind of inconsistent (like bounds), should consolidate into one node

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