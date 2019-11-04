# MIL Missions

## Introduction
MIL Missions is a system for managing the highest level decision making programs in a ROS robot. For example, a robotic boat might have the following missions programmed: "Station Hold", "Remote Control", "Dock", "Move to Waypoint", etc. At any time, one of these missions will be running, interface with the platform'ss GNC and perception systems to achieve a certain goal.

The system is server-client based, using actionlib, with one server managing the current mission and any number of clients connected to monitor the mission, cancel the mission, or start a new mission.

Missions are written in python as classes inheriting from a base class. The system's base class provides member functions / variables for the systems's current state, perception nodes, and actuators. To provide asynchronous execution, missions use the alternative ROS client library txros.

## Usage
### GUI
![screenshot](https://user-images.githubusercontent.com/9502636/31801632-732b6258-b517-11e7-9064-80464f47f26a.png)
MIL Missions provides an rqt plugin to monitor, cancel, and trigger new missions.

To run the GUI, open rqt and add the plugin or from the terminal run:
```rqt -s mil_missions_gui.Dashboard```

## CLI
MIL Missions provides a command line interface to trigger a new mission, list available missions, and cancel the current mission.

### Running a new mission
To run the Wait mission (which simply sleeps for a specified time) with parameters "5" (to run for 5 seconds)
```runmission Wait 5```

### Listing available missions
To see what missions are available, run
```listmissions```

### Canceling the current mission
To stop execution of the current mission prematurely, run
```cancelmission```

## Integrating with a new Robot
To use MIL missions on a new Robotic platform, you need to do the following:
1. Create a class extending mil_missions_core.BaseMission, providing interfaces to your robot's systems through ROS
1. Write a number of missions, extending the new base mission for various things you want your robot to do. Each of these can take arguments to extend their functionality
1. Create a launch file to run the mil_missions server with our base class and missions

### Implementing a BaseClass
Here is an example BaseClass implementation:
```
#!/usr/bin/env python
from mil_missions_core import BaseMission
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from my_robot.msg import MoveToAction, MoveToGoal
from txros import util


class MyRobotBaseMission(BaseMission):
    @util.cancellableInlineCallbacks
    @classmethod
    def _init(cls, mission_runner):
        super(ExampleBaseMission, cls)._init(mission_runner)  # Must be done to have access to node handle
        # Establish robot specific subscribers, tools, etc here
        cls.pose = None
        cls.vision_target_pose = None
        cls._odom_sub = cls.nh.subscribe('/odom', Odemetry, cls.pose_cb)
        cls._vision_target_sub = cls.nh.subscribe('/vision_target', Pose, cls.vision_target_cb)
        cls._move_to_client = action.ActionClient(cls.nh, '/move_to', MoveToAction)
        print 'Waiting for MoveTo server'
        yield cls._move_to_client.wait_for_server()
        print 'Connected to MoveTo server!'

    @classmethod
    def move_to(cls, pose):
        goal = cls._move_to_client.send_goal(MoveToGoal(pose=pose))
        return goal.get_result()
        

    @classmethod
    def pose_cb(cls, odom):
        cls.pose = odom.pose.pose

    @classmethod
    def vision_target_cb(cls, pose):
        cls.vision_target_pose = pose
```
Some things to note with the above example:
* ExampleBaseMission must extend mil_missions_core.BaseMission
* The ```_init(cls, mission_runner)``` function should first call the mil_missions_core.BaseMission ```_init``` function so it can store the mission servers node handle object (used for ROS interactions), ```super(ExampleBaseMission, cls)._init(mission_runner)```
* The ```@util.cancellableInlineCallbacks``` decorator must be used for any functions producing twisted generators (they use the ```yield``` keyword).
* The ```_init(cls)``` function is called once when the mission server is started. It should be used to subscribe to the robot's systems, set up actionclients, service clients, initialize state variables, etc.
* All functions/variables defined for the base mission should have the ```@classmethod``` decorator. This means that inherited classes can share these without having to copy or create new ones

### Writing a Mission
Here is an example of a Mission for the same example system we used above:
```
from twisted.internet import defer
from txros import util
from my_robot_base_mission import MyRobotBaseMission
from geometry_msgs.msg import Position, Quaternion, Pose


class MoveToWaypoint(MyRobotBaseMission):
    @util.cancellableInlineCallbacks
    def run(self, parameters):
        if 'position' not in parameters:
            raise Exception('No position in parameters')
        if 'orientation' not in parameters:
            parameters['orientation'] = Quaternion(0, 0, 0, 1)
        pose = Pose(position=parameters['position'], orientation=parameters['orientation'])
        self.send_feedback('Moving to position {}'.format(pose.position))
        self.send_feedback('Moving to orientation {}'.format(pose.orientation))
        result = yield self.move_to(pose)
        if not result.success:
            raise Exception('Error moving, {}'.format(result.failure_reason)
        defer.returnValue('Move completed!')
	   
```
Again, some things to note:
* The mission inherits ExampleBaseMission, which allows it to use the move_to function
* The mission's run function takes in an argument ```parameters```, which by default is a json decoded object from the string passed to the mission goal.
* The mission raises exceptions if something goes wrong. When the server sees an exception is raised, it aborts the mission and sends the error message as the result
* Again, the run function has ```@util.cancellableInlineCallbacks``` because it waits on the move goal to complete
* The mission uses ```self.send_feedback```, which sends a string to connected clients about what the mission is doing/thinking about right now
* The mission does very little computation, it just interfaces with other ROS systems. This is how missions are intended to work. Any serious computation should be done in other nodes.

### Launching the mission runner
Now that we have a base mission and at least one mission, we can run the mission server using this new system. For this, lets use a launch file so we don't need to type the parameters each time.
```
<launch>
    <node pkg="mil_missions" type="mission_runner" name="mission_runner">
        <param name="missions_module" value="myrobot_missions" />
        <param name="base_mission" value="MyRobotBaseMission" />
    </node>
</launch>
```
This assumes the above code was written in a python module called ```myrobot_missions```, which contains the ```MyRobotBaseMission``` we created earlier. When this launch file is run, the mission server does the following:
* Connects to ros, creates a txros node handle object
* Imports ```MyRobotBaseMission```
* Calls ```MyRobotBaseMission._init(self)```, passing it the mission runner object (which contains the node handle) to the base mission. If this init function fails, the mission server will crash
* Imports all other classes in the ```myrobot_missions``` module which extend ```MyRobotBaseMission```, including our example ```MoveToWaypoint``` mission
* Calls the ```init()``` function for each mission, if it exists. 
* Waits for a new mission to be triggered from the CLI or GUI (see usage guide above)


