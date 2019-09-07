#!/usr/bin/env python
"""Mission Planner Module that uses a DAG and YAML to find which mission to perform."""
from txros import util, NodeHandle
from twisted.internet import defer
from navigator_singleton.navigator import Navigator
import nav_missions
import nav_missions_test
import Queue as que
import sys
from Queue import Queue
from sets import Set
from navigator_tools import DBHelper, MissingPerceptionObject
import genpy
from std_msgs.msg import String
from mil_misc_tools.text_effects import fprint
__author__ = "Tess Bianchi"


class Mission(object):
    """The class that represents a mission."""

    def __init__(self, name, item_dep, timeout):
        """Initialize a Mission object."""
        self.name = name
        self.item_dep = item_dep
        self.children = []
        self.timeout = timeout

    def add_child(self, child):
        """Add child to a mission."""
        self.children.append(child)

    @util.cancellableInlineCallbacks
    def do_mission(self, navigator, planner, module):
        """Perform this mission."""
        fprint(self.name, msg_color="green", title="STARTING MISSION:")
        to_run = getattr(module, self.name)
        yield to_run.main(navigator)

    @util.cancellableInlineCallbacks
    def safe_exit(self, navigator, err, planner, module):
        """Run a safe exit of a mission."""
        try:
            to_run = getattr(module, self.name)
            if hasattr(to_run, 'safe_exit'):
                yield to_run.safe_exit(navigator, err)
            else:
                fprint("Hmmmm. This isn't good. Your mission failed, and there was no safe exit. "
                       "I hope this mission doesn't have any children.", msg_color="red")
        except Exception as exp:
            print exp
            fprint("Oh man this is pretty bad, your mission's safe exit failed. SHAME!", msg_color="red")


class MissionPlanner:
    """The class that plans which mission to do next."""

    def __init__(self, yaml_text, mode='r'):
        """Initialize the MissionPlanner class."""
        self.tree = []
        self.queue = Queue()
        self.found = Set()

        self.base_mission = None
        self.completed_mission = 0
        self.total_mission_count = len(yaml_text.keys())
        if mode == 'r':
            self.module = nav_missions
        elif mode == 't':
            self.module = nav_missions_test
        else:
            raise Exception("Valid Mode not selected")

        self.running_mission = False
        self.mission_defer = None

        self.current_mission_name = None
        self.current_mission = None
        self.current_mission_timeout = None
        self.current_mission_start_time = None

        self.keep_running = True

        self.load_missions(yaml_text)
        if self.base_mission is not None:
            self.total_mission_count -= 1

    @util.cancellableInlineCallbacks
    def init_(self, sim_mode=False):
        """Initialize the txros aspects of the MissionPlanner."""
        self.nh = yield NodeHandle.from_argv("mission_planner")
        self.navigator = yield Navigator(self.nh)._init(sim_mode)

        self.helper = yield DBHelper(self.nh).init_()
        yield self.helper.begin_observing(self.new_item)
        self.pub_msn_info = yield self.nh.advertise("/mission_planner/mission", String)
        yield self.nh.sleep(1)

        # This needs to be called in case begin_observing doesn't call refresh
        self.refresh()
        defer.returnValue(self)

    def load_missions(self, yaml_text):
        """Load all the missions from the YAML file into the structures used in the DAG."""
        # Load all the missions into a dict mission_name -> Mission
        # If this mission doesn't have any mission deps, add it to the tree
        my_missions = {}
        mission_to_mission_dep = {}

        for name in yaml_text.keys():
            mission = yaml_text[name]
            obj_dep = mission["object_dep"]
            mis_dep = mission["mission_dep"]
            is_base = mission["is_base"]
            timeout = mission["timeout"]
            if timeout == 'inf':
                timeout = sys.maxint
            m = Mission(name, obj_dep, timeout)
            my_missions[name] = m
            if is_base:
                self.base_mission = m
            elif mis_dep == "None":
                self.tree.append(m)
            else:
                mission_to_mission_dep[name] = mis_dep

        # Go through the missions and give them all children dependencies
        for mission_name in mission_to_mission_dep.keys():
            parent_name = mission_to_mission_dep[mission_name]
            child_name = mission_name
            parent_mission = my_missions[parent_name]
            child_mission = my_missions[child_name]
            parent_mission.add_child(child_mission)

    def _is_in_queue(self, x):
        with self.queue.mutex:
            return x in self.queue.queue

    def new_item(self, obj):
        """
        Callback for a new object being found.

        ASYNCHRONOUS
        """
        fprint("NEW ITEM: {}".format(obj.name), msg_color="blue")
        if self.base_mission is not None and self.current_mission_name is self.base_mission.name:
            self.mission_defer.cancel()
        self.found.add(obj.name)
        self.refresh()

    def refresh(self):
        """
        Called when the state of the DAG needs to be updated due to a mission completing or an object being found.

        CALLED ASYNCHRONOUS
        """
        for mission in self.tree:
            if self.can_complete(mission) and not self._is_in_queue(mission) and mission.name != self.current_mission_name:
                fprint("mission: {}".format(mission.name), msg_color="blue", title="ADDING")
                self.queue.put(mission)

    def can_complete(self, mission):
        """
        Figure out if a mission can be completed.

        CALLED ASYNCHRONOUS
        """
        for item in mission.item_dep:
            if item not in self.found:
                return False
        return True

    @util.cancellableInlineCallbacks
    def do_mission(self, mission):
        """Perform a mission, and ensure that all of the post conditions are enforced."""
        m = mission.do_mission(self.navigator, self, self.module)
        # TOD: do something with the result of the mission
        yield m

    def _mission_complete(self, mission):
        self.tree.remove(mission)
        self.current_mission_name = None
        for m in mission.children:
            self.tree.append(m)
            self.refresh()
        self.completed_mission += 1
        if self.completed_mission == self.total_mission_count:
            fprint("ALL MISSIONS COMPLETE", msg_color="green")
            self.keep_running = False

    def _object_gone_missing(self, missing_objects):
        fprint("This object {} is no longer in the list".format(missing_objects), msg_color="red")
        self.helper.stop_ensuring_object_permanence()
        for o in missing_objects:
            if o in self.found:
                self.helper.remove_found(o)
                self.found.remove(o)
        self.mission_defer.cancel()

    def _err_base_mission(self, err):
        self.running_mission = False
        self.current_mission_name = None
        if err.type == defer.CancelledError:
            fprint("Base mission cancelled", msg_color="red", title="BASE MISSION ERROR:")
        else:
            fprint(err, msg_color="red", title="BASE MISSION ERROR:")

    def _end_base_mission(self, result):
        self.running_mission = False
        self.current_mission_name = None
        fprint(result, msg_color="green", title="BASE MISSION COMPLETE:")

    @util.cancellableInlineCallbacks
    def _err_mission(self, err):
        self.running_mission = False
        self.current_mission_name = None
        self.helper.stop_ensuring_object_permanence()
        self.current_mission_timeout = None
        if err.type == defer.CancelledError:
            fprint("Mission Canceled", msg_color="red",
                   title="{} MISSION ERROR: ".format(self.current_mission_name))
            defer.returnValue(True)
        elif err.type == MissingPerceptionObject:
            if err.value.missing_object in self.found:
                self.helper.remove_found(err.value.missing_object)
                self.found.remove(err.value.missing_object)
            fprint("Missing Perception Object thrown", msg_color="red",
                   title="{} MISSION ERROR: ".format(self.current_mission_name))
            defer.returnValue(True)
        else:
            fprint(err, msg_color="red", title="{} MISSION ERROR: ".format(self.current_mission_name))
        yield self.current_mission.safe_exit(self.navigator, err, self, self.module)
        self._mission_complete(self.current_mission)
        self.current_mission = None

    def _end_mission(self, result):
        self.pub_msn_info.publish(String("Ending Mission {}".format(self.current_mission_name)))
        fprint(str(result) + " TIME: " + str((self.nh.get_time() - self.current_mission_start_time).to_sec()),
               msg_color="green", title="{} MISSION COMPLETE: ".format(self.current_mission_name))
        self.running_mission = False
        self.current_mission_name = None
        self.helper.stop_ensuring_object_permanence()
        self._mission_complete(self.current_mission)
        self.current_mission = None
        self.current_mission_timeout = None

    @util.cancellableInlineCallbacks
    def _run_mission(self, mission):
        self.pub_msn_info.publish(String("Starting Mission {}".format(mission.name)))
        yield self.nh.sleep(.3)
        self.running_mission = True
        self.current_mission_name = mission.name
        self.current_mission = mission
        self.helper.ensure_object_permanence(mission.item_dep, self._object_gone_missing)
        self.mission_defer = self.do_mission(mission)
        self.current_mission_timeout = genpy.Duration(mission.timeout)
        self.current_mission_start_time = self.nh.get_time()
        self.mission_defer.addCallbacks(self._end_mission, errback=self._err_mission)

    def _run_base_mission(self):
        if self.base_mission is not None:
            self.running_mission = True
            self.current_mission_name = self.base_mission.name
            self.mission_defer = self.do_mission(self.base_mission)
            self.mission_defer.addCallbacks(self._end_base_mission, errback=self._err_base_mission)

    @util.cancellableInlineCallbacks
    def empty_queue(self):
        """Constantly empties the queue if there is something in it, or run the base mission otherwise."""
        starting_time = self.nh.get_time()
        while self.keep_running:
            if self.current_mission_timeout is not None:
                if (self.nh.get_time() - self.current_mission_start_time) > self.current_mission_timeout:
                    fprint(self.current_mission_name, msg_color="red", title="MISSION TIMEOUT:")
                    self.mission_defer.cancel()
            if self.running_mission:
                self.refresh()
                yield self.nh.sleep(1)
                continue
            try:
                mission = self.queue.get(block=False)
                yield self._run_mission(mission)
            except que.Empty:
                self._run_base_mission()
            except Exception as exp:
                fprint(exp, msg_color="red", title="TESS SUBUKU")
            finally:
                self.refresh()
                yield self.nh.sleep(1)

        fprint("TOTAL RUN TIME: {}".format((self.nh.get_time() - starting_time).to_sec()), msg_color="green")
