Culture
=======
The Machine Intelligence Lab has a unique student culture which creates a synergistic environment dedicated to the study and development of intelligent, autonomous robots. Without an application process, new members are encouraged to show up in MIL. We believe that learning occurs best by being inquisitive, and therefore we continually encourage all members to ask fellow MILers about their ongoing projects and tasks in order to learn about Subjugator and Navigator.

Meetings
--------
As of the Spring 2022 semester, each team has its own meeting schedules and strategies
for completing projects.

The software team currently meets on Mondays and Fridays, and software members
are highly encouraged to come. In these meetings, software members exchange ideas
with each other and help get new members set up with MIL. Software members who
are not able to attend the meetings can reach out to any lead to get information
about what was shared in the meeting.

Diversity, Equity, and Inclusion
--------------------------------
MIL has always been, and will always be, strongly committed to diversity,
equity, and inclusion.

Glossary
--------
This page lists common words / acronyms uses in MIL and robotics more generally. This should help get past some of our jargon.

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
    The :term:`AUVSI`-run competition which :term:`NaviGator` competes in.

  VRX
    A competition named Virtual :term:`RobotX`. Similar to RobotX, but no physical
    components or travel are involved.

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

  POE
     Power-Over-Ethernet, a variety of protocols to provide
     power over an ethernet cable, so that the same cable can be used
     to both power and transmit data.
