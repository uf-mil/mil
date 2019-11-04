Glossary
========
This page lists common words / acrynomys uses in MIL and robotics more generally. This should help get passed some of our jargon.


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
    The programs which determine what is in the enviroment around a robot.
    Often uses cameras, lidar, radar, etc.

  State Estimation
    The process / program which determines where we are in the world.
    Usually fuses multiple sensors ( :term:`DVL` , gps, imu) to
    produce an estimate of the robot's :term:`pose`

  DVL
    A dopler velocity log. Is used primarily in underwater
    systems to measure the linear velocity of the robot.

  RobotX
    The :term:`AUVSI`-run competition which :term:`NaviGator` competes in

  NaviGator
    An autnomous boat developed by MIL.

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
