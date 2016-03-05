Kill\_Handling
==============

Kill handling software

In this package there is:
* Kill Master: manages all kills
* Ease of use scripts: for killing, clearing kills
* A node to kill when a condition is met
To use this node properly first launch the kill_master. To have a node listen to the kill master use a kill listener see usage below. To publish kills use a broadcaster see usage below.

Listeners can be started without the kill master but they will always output true for killed since the kill master is inactive. This also means that if the kill master ever unexpectidly shuts down all kill listeners will kill there respective nodes.

## Usage
Command line
```
# Run the master
$ rosrun kill_handling kill_master
    
# Cmd line kill
$ rosrun kill_handling kill

# Cmd line kill with specific id
$ rosrun kill_handling kill my_id

# Cmd line clear all
$ rosrun kill_handling clear

# Cmd line clear specific ids
$ rosrun kill_handling clear id1 id2 ...
```

Aliases in the [bashrc](https://github.com/uf-mil/uf-mil/blob/master/bashrc)
```
alias k="rosrun kill_handling kill"
alias clc_k="rosrun kill_handling clear"
function list_kills(){

rostopic echo /kill &
sleep 1
kill -SIGINT $!

}
```

The kill_on_cond node can be used to kill based on a condition specified in the cond param evaluate on subscriber callbacks to a topic that should be remaped from topic to _your topic_. The value of the cond param should be a python code executable through the eval builtin function. See [kill_on_cond](https://github.com/uf-mil/software-common/blob/master/kill_handling/scripts/kill_on_cond) for more info. Below is an example from [here](https://github.com/uf-mil/SubjuGator/blob/master/sub_launch/launch/common.xml).
```
<node pkg="kill_handling" type="kill_on_cond" name="height_over_bottom_kill">
    <remap from="topic" to="dvl/range"/>
    <param name="cond" value="data &lt; .25"/>
  </node>
```

C++ listener
```Cpp
#include <kill_handling/listener.h>
kill_handling::KillListener kill_listener(/*on_kill_cb*/, /*on_unkill_cb*/);

// Get killed status as a bool
bool killed = kill_listener.get_killed();

// Get the descriptions of the kill or kills
std::vector<std::string> reasons = kill_listener.get_kills();
```

C++ broadcaster
```Cpp
#include <kill_handling/broadcaster.h>
kill_handling::KillBroadcaster kill_broadcaster("id", "description");

// Send out an active kill msgs
kill_broadcaster.send(true);

// Send out a non-active kill msgs
kill_broadcaster.send(false);

// Clear the kill 
kill_broadcaster.clear();
```

python listener
```python
from kill_handling.listener import KillListener
kill_listener = KillListener(set_kill_callback, clear_kill_callback)

# Get killed status as a Bool
killed = kill_listener.get_killed()

# Get a list of reasons for the kill
reasons = kill_listener.get_kills()
```

python broadcaster
```python
from kill_handling.broadcaster import KillBroadcaster
kill_broadcaster = KillBroadcaster(id='id', description='description')

# Send out an active kill msgs
kill_broadcaster.send(True)

# Send out a non-active kill msgs
kill_broadcaster.send(False)

# Clear the kill 
try:
    kill_broadcaster.clear()
except rospy.service.ServiceException, e:
    rospy.logwarn(str(e))
```

## TODO
Added an ignore list to the listeners such that nodes can ignore particular kills if they choose to.

## Issues
