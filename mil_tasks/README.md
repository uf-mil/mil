# MIL Tasks

## Introduction
MIL Tasks is a system for managing the highest level decision making programs in a ROS robot. For example, a robotic boat might have the following tasks programmed: "Station Hold", "Remote Control", "Dock", "Move to Waypoint", etc. At any time, one of these tasks will be running, interface with the platform'ss GNC and perception systems to achieve a certain goal.

The system is server-client based, using actionlib, with one server managing the current task and any number of clients connected to monitor the task, cancel the task, or start a new task.

Tasks are written in python as classes inheriting from a base class. The system's base class provides member functions / variables for the systems's current state, perception nodes, and actuators. To provide asynchronous execution, tasks use the alternative ROS client library txros.

## Usage
### GUI
![screenshot](https://user-images.githubusercontent.com/9502636/31801632-732b6258-b517-11e7-9064-80464f47f26a.png)
MIL Tasks provides an rqt plugin to monitor, cancel, and trigger new tasks.

To run the GUI, open rqt and add the plugin or from the terminal run:
```rqt -s mil_tasks_gui.Dashboard```

## CLI
MIL Tasks provides a command line interface to trigger a new task, list available tasks, and cancel the current task.

### Running a new task
To run the Wait task (which simply sleeps for a specified time) with parameters "5" (to run for 5 seconds)
```runtask Wait 5```

### Listing available tasks
To see what tasks are available, run
```listtasks```

### Canceling the current task
To stop execution of the current task prematurely, run
```canceltask```

## Integrating with a new Robot
To use MIL tasks on a new Robotic platform, you need to do the following:
1. Create a class extending mil_tasks_core.BaseTask, providing interfaces to your robot's systems through ROS
1. Write a number of tasks, extending the new base task for various things you want your robot to do. Each of these can take arguments to extend their functionality
1. Create a launch file to run the mil_tasks server with our base class and tasks

### Implementing a BaseClass
Here is an example BaseClass implementation:
```
#!/usr/bin/env python
from mil_tasks_core import BaseTask
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from my_robot.msg import MoveToAction, MoveToGoal
from txros import util


class MyRobotBaseTask(BaseTask):
    @util.cancellableInlineCallbacks
    @classmethod
    def _init(cls, task_runner):
        super(ExampleBaseTask, cls)._init(task_runner)  # Must be done to have access to node handle
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
* ExampleBaseTask must extend mil_tasks_core.BaseTask
* The ```_init(cls, task_runner)``` function should first call the mil_tasks_core.BaseTask ```_init``` function so it can store the task servers node handle object (used for ROS interactions), ```super(ExampleBaseTask, cls)._init(task_runner)```
* The ```@util.cancellableInlineCallbacks``` decorator must be used for any functions producing twisted generators (they use the ```yield``` keyword).
* The ```_init(cls)``` function is called once when the task server is started. It should be used to subscribe to the robot's systems, set up actionclients, service clients, initialize state variables, etc.
* All functions/variables defined for the base task should have the ```@classmethod``` decorator. This means that inherited classes can share these without having to copy or create new ones

### Writing a Task
Here is an example of a Task for the same example system we used above:
```
from twisted.internet import defer
from txros import util
from my_robot_base_task import MyRobotBaseTask
from geometry_msgs.msg import Position, Quaternion, Pose


class MoveToWaypoint(MyRobotBaseTask):
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
* The task inherits ExampleBaseTask, which allows it to use the move_to function
* The task's run function takes in an argument ```parameters```, which by default is a json decoded object from the string passed to the task goal.
* The task raises exceptions if something goes wrong. When the server sees an exception is raised, it aborts the task and sends the error message as the result
* Again, the run function has ```@util.cancellableInlineCallbacks``` because it waits on the move goal to complete
* The task uses ```self.send_feedback```, which sends a string to connected clients about what the task is doing/thinking about right now
* The task does very little computation, it just interfaces with other ROS systems. This is how tasks are intended to work. Any serious computation should be done in other nodes.

### Launching the task runner
Now that we have a base task and at least one task, we can run the task server using this new system. For this, lets use a launch file so we don't need to type the parameters each time.
```
<launch>
    <node pkg="mil_tasks" type="task_runner" name="task_runner">
        <param name="tasks_module" value="myrobot_tasks" />
        <param name="base_task" value="MyRobotBaseTask" />
    </node>
</launch>
```
This assumes the above code was written in a python module called ```myrobot_tasks```, which contains the ```MyRobotBaseTask``` we created earlier. When this launch file is run, the task server does the following:
* Connects to ros, creates a txros node handle object
* Imports ```MyRobotBaseTask```
* Calls ```MyRobotBaseTask._init(self)```, passing it the task runner object (which contains the node handle) to the base task. If this init function fails, the task server will crash
* Imports all other classes in the ```myrobot_tasks``` module which extend ```MyRobotBaseTask```, including our example ```MoveToWaypoint``` task
* Calls the ```init()``` function for each task, if it exists. 
* Waits for a new task to be triggered from the CLI or GUI (see usage guide above)


