Glossary
========
This page lists common words / acronyms uses in MIL and robotics more generally. This should help get past some of our jargon.

Some are technical while others are just insides jokes.


.. glossary::
  :sorted:

  mission
    The program that makes high level decision making to complete a
    given task. Sets :term:`waypoint` using information from :term:`perception`

  waypoint
    A new :term:`pose` for the robot to attempt to get to

  pose
    A position (point in the world) and orientation (rotation in the world)

  perception
    The program(s) which attempt to understand what is in the environment around a robot.
    Often uses cameras, lidar, radar, etc.

  State Estimation
    The process / program which determines where we are in the world.
    Usually fuses multiple sensors ( :term:`DVL` , gps, imu) to
    produce an estimate of the robot's :term:`pose`

  DVL
    A Doppler velocity log. Is used primarily in underwater
    systems to measure the linear velocity of the robot.

  RobotX
    The :term:`AUVSI`-run competition which :term:`NaviGator` competes in

  NaviGator
    An autonomous boat developed by MIL.

  AUVSI
    A non-profit organization which runs several competitions which MIL participates in

  Zobelisk
    A powerful computer in MIL used to run simulation tasks. Named after former member
    David Zobel

  Shuttle
    A powerful computer in MIL used to run simulation tasks. Has been mostly replaced
    by :term:`Zobelisk`

  Kevbot
    A funny name for the help page in these docs, named so because it was written
    by Kevin (kev-the-dev) to try to automate helping new programmers

  node
    A program running on a robot. Usually performs a very specific purpose

  alarms
    Global Boolean state shared by many nodes, such as :term:`kill` or autonomous

  kill
    A global state indicating the the robot cannot move / actuate. This is largely
    a safety feature. The robot can usually be killed remotely

  bag
    A file which stores a set of ROS messages received over a certain interval.
    Used to collect data to later play back for testing / debugging

  Drew Bagnell
    An external USB hard-drive used to store :term:`bag` files.
    Named after former member and legend Drew Bagnell, now CTO of Aurora Innovation

  root
    1. The administrator / superuser account on Linux. Has permission to do anything on the system
    2. The top most directory or a project or the entire filesystem. For example,
       the root of the repository is installed to ~/catkin_ws/src/mil

  Food train
    An spontaneous event where people working in MIL go out somewhere to eat

  Langford
    (verb) Doing someone else's work before they complete it and without them knowing you were doing so

  Feasible
     A solution, though possibly not the best solution, to a problem.
     In MIL, is used as a joke for a solution that technically meets
     the criteria but does so in an unexpected or naive way
