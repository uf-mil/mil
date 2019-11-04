Sub Mission Manager
==================

# Running the sub

```shell

    # If you're not the sub
    rosparam set /draw false  # Set to true if you want visualization
    roslaunch sub8_gazebo duck.launch
```

# Running shore control (3d mouse)

```shell
    roslaunch sub8_launch shore_control.launch
```

To get results that are useful, you'll want to set up rviz to visualize tf, /wrench, /posegoal, and /odom (Set odom and posegoal to use axes instead of arrow). I use /odom, keeping a bunch of arrows, for some indication that movement has actually happened.

Orintation of axis markers with respect to sub:
	Red is forward, Green is left, Blue is up

With the wire facing away from you, right-button will snap the posegoal to the sub's current pose, and left-button will make the sub go to the current posegoal

tags for greppers: 3dmouse 3d-mouse 3dconnexion


# Running a Mission

Here are some examples of missions you might run. Ex: "stop" stops the sub where it is, forward_1_m goes forward 1 meter, level_off zeros roll and pitch.

```shell
    rosrun sub8_missions tx_mission surface

    rosrun sub8_missions tx_mission level_off

    rosrun sub8_missions tx_mission tx_test
```


# Why we're using TXROS

* We want to be able to wait for the next message from a topic without having to loop-and-wait

* We want to be able to do things asynchronously

* We want to be able to externally enforce timeouts

* Waiting for things to be ready is much more intelligent (Than initializing with None and then polling)

# Tutorial

Create a file in missions called "my_mission.py" (Or whatever you want). Add it to the __init__.py file in missions.
In that file, write a run, like this (Be sure to use @util.cancellableInlineCallbacks)

```python
    # move_right.py

    from txros import util
    @util.cancellableInlineCallbacks
    def run(sub):
        yield sub.move.right(5).go()
        print "Done"
```

`sub.move` creates a `PoseEditor`, one of Forrest's ideas. The pose editor can be "edited", i.e. `move.right(5)` to move the sub right 5 meters, or `move.depth(3)`, to set z to -3. `move.go()` returns a twisted object called a **Deferred**. When you *yield* a `Deferred`, the script will stop executing until that deferred completes. So in the above example, "Done" will not be printed until the sub has moved right 5 meters. If you *don't*, yield, the rest of your script will continue to execute asynchronously. Let's look at another example, with no yield.

```python
    # move_right_asynchronous.py

    from txros import util
    @util.cancellableInlineCallbacks
    def run(sub):
        sub.move.right(5).go()
        print "Done"
```

Here, "Done" will print immediately after the mission starts running (Not waiting for the sub to move), and the sub will **continue to move** until it has gotten 5 meters to the right. Externally, the mission runner will wait for the whole mission script to finish before moving on.

We haven't fully decided how vision data, etc will be shared with the mission manager, but at its core, it will be exposed as a `Deferred` inside of `sub8/tx_sub.py`. Here's how that might look.


```python
    # bump_buoy.py

    from txros import util
    @util.cancellableInlineCallbacks
    def run(sub):
        # buoy_search is a Deferred
        buoy_search = look_for_buoy('red')
        print "We're looking for a buoy, executing a scan pattern"

        yield sub.move.right(5).go()
        # Wait until we finish moving right (We yielded the Deferred)
        yield sub.move.down(1).go()
        # Wait until we finish moving down (We yielded the Deferred)

        # Now we'll wait until buoy_search poops out a result
        buoy_location = yield buoy_search

        # let's assume buoy_location is ~ np.array([x, y, z])
        yield sub_singleton.move.height(buoy_location[2]).go()
        # Wait until we finish matching the buoy's height (We yielded the Deferred)
        yield sub_singleton.move.set_position(buoy_location).go()
        # Wait until we match the buoy's position, bumping it

        # back off 2m
        yield sub_singleton.move.backward(2).go()
        # Wait until we have backed off

        print "Bumped the buoy"
```


# TODO
    - Chaining together multiple missions in the command line
    - Responding to sensor input
    - Intelligently using motion planning (instead of simply using moveto)
    - Allow missions to have more control over failure conditions, etc
    - Allow for time-out conditions (Enforce them, if need-be)
