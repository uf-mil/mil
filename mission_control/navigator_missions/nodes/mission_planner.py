#!/usr/bin/env python
"""Mission Planner Module that uses a DAG and YAML to find which mission to perform."""
from txros import util, NodeHandle
from twisted.internet import defer, reactor
from navigator_singleton.navigator import Navigator
import nav_missions
import Queue as que
from Queue import Queue
from sets import Set
from navigator_tools import DBHelper
import yaml
import os
__author__ = "Tess Bianchi"


class Mission(object):
    """The class that represents a mission."""

    def __init__(self, name, item_dep):
        """Initialize a Mission object."""
        self.name = name
        self.item_dep = item_dep
        self.children = []
        self.currently_running_mission = False

    def add_child(self, child):
        """Add child to a mission."""
        self.children.append(child)

    @util.cancellableInlineCallbacks
    def do_mission(self, navigator, planner):
        """Perform this mission."""
        print "starting mission:", self.name
        try:
            to_run = getattr(nav_missions, self.name)
            yield to_run.main(navigator)
        except Exception:
            print "This mission doesn't exist"

    def safe_exit(self, navigator, err, planner):
        """Run a safe exit of a mission."""
        try:
            to_run = getattr(nav_missions, self.name)
            yield to_run.safe_exit(navigator, err)
        except Exception:
            print "This mission doesn't exist"


class MissionPlanner:
    """The class that plans which mission to do next."""

    def __init__(self, yaml_text):
        """Initialize the MissionPlanner class."""
        self.tree = []
        self.queue = Queue()
        self.found = Set()
        self.completeing_mission = False
        self.base_mission = None
        # TODO Put in YAML file

        self.load_missions(yaml_text)

    @util.cancellableInlineCallbacks
    def init_(self):
        """Initialize the txros aspects of the MissionPlanner b."""
        self.nh = yield NodeHandle.from_argv("mission_planner")
        self.navigator = yield Navigator(self.nh)._init(True)

        helper = yield DBHelper(self.nh).init_()
        yield helper.begin_observing(self.new_item)

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
            m = Mission(name, obj_dep)
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
        """Callback for a new object being found."""
        print "new item, {}".format(obj.name)
        self.found.add(obj.name)
        self.refresh()

    def refresh(self):
        """Called when the state of the DAG needs to be updated due to a mission completing or an object being found."""
        for mission in self.tree:
            if self.can_complete(mission) and not self._is_in_queue(mission):
                print "adding mission:", mission.name
                self.queue.put(mission)

    def can_complete(self, mission):
        """Figure out if a mission can be completed."""
        for item in mission.item_dep:
            if item not in self.found:
                return False
        return True

    @util.cancellableInlineCallbacks
    def do_mission(self, mission):
        """Perform a mission, and ensure that all of the post conditions are enforced."""
        # TODO: have a better way to make sure all post conditions are enforced
        m = mission.do_mission(self.navigator, self)
        if hasattr('mission', 'safe_exit'):
            m.addErrback(lambda err: mission.safe_exit(self.navigator, err))
        result = yield m
        print result

    def check_current_mission_fail(self):
        pass

    @util.cancellableInlineCallbacks
    def empty_queue(self):
        """Constantly empties the queue if there is something in it, or run the base mission otherwise."""
        while True:
            if self.currently_running_mission:
                self.check_current_mission_fail()
                yield self.nh.sleep(.3)
                continue
            try:
                mission = self.queue.get(block=False)
                self.current_mission = self.do_mission(mission)
            except que.Empty:
                if self.base_mission is not None:
                    yield self.do_mission(self.base_mission)
            else:
                # TODO: create a different exception for when the missions fails, and act appropriately
                # TODO: constantly check for changing classified objects and abort mission if the object changes.
                # The mission succeeded! Yay!
                self.tree.remove(mission)
                for m in mission.children:
                    self.tree.append(m)
                    self.refresh()
            yield self.nh.sleep(.3)


@util.cancellableInlineCallbacks
def main():
    """The main method."""
    yaml_text = None
    yaml_file = "missions.yaml"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    yaml_file = dir_path + "/" + yaml_file

    with open(yaml_file, 'r') as stream:
        try:
            yaml_text = yaml.load(stream)
            planner = yield MissionPlanner(yaml_text).init_()
            planner.empty_queue()
        except yaml.YAMLError as exc:
            print(exc)


reactor.callWhenRunning(main)
reactor.run()
