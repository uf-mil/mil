#!/usr/bin/env python
"""Mission Planner Module that uses a DAG and YAML to find which mission to perform."""
from txros import util, NodeHandle
from twisted.internet import defer, threads, reactor
from navigator_singleton.navigator import Navigator
import nav_missions
import nav_missions_test
from navigator_tools import MissingPerceptionObject, DBHelper
import navigator_tools as nt
from std_msgs.msg import String
from navigator_tools import fprint
from timeout_manager import TimeoutManager
__author__ = "Tess Bianchi"


class Mission(object):
    """The class that represents a mission."""

    def __init__(self, name, item_dep, min_time, weight, points, mission_script=None):
        """Initialize a Mission object."""
        self.name = name
        self.item_dep = item_dep
        self.children = []
        self.min_time = min_time
        self.weight = weight
        self.points = points
        self.attempts = 0
        self.timeout = None
        self.start_time = None
        self.mission_script = mission_script

    def add_child(self, child):
        """Add child to a mission."""
        self.children.append(child)

    @util.cancellableInlineCallbacks
    def do_mission(self, navigator, planner, module, *args, **kwargs):
        """Perform this mission."""
        if self.mission_script:
            to_run = getattr(module, self.mission_script)
        else:
            to_run = getattr(module, self.name)
        fprint(self.name, msg_color="green", title="STARTING MISSION")
        res = yield to_run.main(navigator, self.attempts, *args, **kwargs)
        defer.returnValue(res)

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

    def __init__(self, mode='r', total_minutes=30):
        """Initialize the MissionPlanner class."""
        self.tree = []
        self.total_time = total_minutes * 60
        self.points = 0
        self.missions_left = []

        self.base_mission = None

        if mode == 'r':
            self.module = nav_missions
        elif mode == 't':
            self.module = nav_missions_test
        else:
            raise Exception("Valid Mode not selected")

        self.current_mission_defer = None
        self.current_mission = None

    @util.cancellableInlineCallbacks
    def init_(self, yaml_text, sim_mode=False):
        self.sim_mode = sim_mode
        assert yaml_text is not None, "YOU NEED A YAML TEXT TO RUN A MISSION"
        """Initialize the txros aspects of the MissionPlanner."""
        self.total_mission_count = len(yaml_text.keys())

        self.nh = yield NodeHandle.from_argv("mission_planner")
        self.navigator = yield Navigator(self.nh)._init(sim_mode)
        self.pub_msn_info = yield self.nh.advertise("/mission_planner/mission", String)
        self.helper = yield DBHelper(self.nh).init_(navigator=self.navigator)

        yield self._load_missions(yaml_text)
        if self.base_mission is not None:
            self.total_mission_count -= 1
        yield self.nh.sleep(1)

        defer.returnValue(self)

    @util.cancellableInlineCallbacks
    def _get_closest_mission(self):
        os = []
        for m in self.tree:
            if m.item_dep is None:
                defer.returnValue(m)
            else:
                os.append(m.item_dep)
        # print[x.name for x in os]
        closest = yield self.helper.get_closest_object(os)
        # print self.tree
        for m in self.tree:
            # print closest.name, m.item_dep.name
            if m.item_dep.name == closest.name:
                defer.returnValue(m)

    @util.cancellableInlineCallbacks
    def _load_missions(self, yaml_text):
        """Load all the missions from the YAML file into the structures used in the DAG."""
        # Load all the missions into a dict mission_name -> Mission
        # If this mission doesn't have any mission deps, add it to the tree
        my_missions = {}
        mission_to_mission_dep = {}
        count = 0

        for name in yaml_text.keys():
            mission = yaml_text[name]
            marker_dep = mission["marker_dep"]
            mis_dep = mission["mission_dep"]
            is_base = mission["is_base"]
            min_time = mission["min_time"]
            points = mission["points"]
            weight = mission["weight"]
            if marker_dep != "None":
                marker = yield self.navigator.database_query(marker_dep, raise_exception=False)
                if not marker.found:
                    fprint("NOT COMPLETING {}, NO MARKER FOUND".format(name), msg_color="green")
                    continue
                marker = marker.objects[0]
            else:
                marker = None

            count += 1
            if "mission_script" in mission.keys():
                mission_script = mission["mission_script"]
                m = Mission(name, marker, min_time, weight, points, mission_script=mission_script)
            else:
                m = Mission(name, marker, min_time, weight, points)
            my_missions[name] = m
            if is_base:
                self.base_mission = m
            elif mis_dep == "None":
                self.missions_left.append(m)
                self.tree.append(m)
            else:
                self.missions_left.append(m)
                mission_to_mission_dep[name] = mis_dep

        # Go through the missions and give them all children dependencies
        for mission_name in mission_to_mission_dep.keys():
            parent_name = mission_to_mission_dep[mission_name]
            child_name = mission_name
            parent_mission = my_missions[parent_name]
            child_mission = my_missions[child_name]
            parent_mission.add_child(child_mission)

        self.total_time -= count * 60

    def _get_time_left(self):
        return self.total_time - (self.start_time - self.nh.get_time()).to_sec()

    @util.cancellableInlineCallbacks
    def _do_mission(self, mission, *args, **kwargs):
        """Perform a mission, and ensure that all of the post conditions are enforced."""
        if "redo" not in kwargs:
            redo = False
        else:
            redo = kwargs["redo"]

        if not redo and not self.sim_mode:
            marker = yield self.navigator.database_query(mission.item_dep)
            yield self.navigator.move.set_position(nt.rosmsg_to_numpy(marker.position)).go()
        mission.start_time = self.nh.get_time()
        mission.attempts += 1
        res = yield mission.do_mission(self.navigator, self, self.module, *args, **kwargs)
        defer.returnValue(res)

    def _mission_complete(self, mission):
        self.current_mission = None
        self.tree.remove(mission)
        for m in mission.children:
            self.tree.append(m)
        self.missions_left.remove(mission)

    @util.cancellableInlineCallbacks
    def _err_mission(self, err):
        if err.type == MissingPerceptionObject or err.type == defer.CancelledError:
            fprint("{} thrown".format(err.type), msg_color="red",
                   title="{} MISSION ERROR: ".format(self.current_mission.name))
            if TimeoutManager.can_repeat(self.missions_left, self._get_time_left(), self.current_mission):
                if err.type == MissingPerceptionObject:
                    new_obj_found = yield self._run_base_mission(self.current_mission.item_dep)
                    if new_obj_found:
                        self.pub_msn_info.publish(String("Retrying Mission {}".format(self.current_mission.name)))
                        self.current_mission.timeout = self.current_mission.min_time
                        yield self._run_mission(self.current_mission, redo=True)
                        defer.returnValue(False)
                    else:
                        fprint("NEW OBJECT NOT FOUND, KILLING MISSION", msg_color="red")
                else:
                    self.pub_msn_info.publish(String("Retrying Mission {}".format(self.current_mission.name)))
                    self.current_mission.timeout = self.current_mission.min_time
                    yield self._run_mission(self.current_mission)
                    defer.returnValue(False)

        else:
            fprint(err, msg_color="red", title="{} MISSION ERROR: ".format(self.current_mission.name))
        self.pub_msn_info.publish(String("Failing Mission {}".format(self.current_mission.name)))
        yield self.current_mission.safe_exit(self.navigator, err, self, self.module)
        self._mission_complete(self.current_mission)

    def _end_mission(self, result):
        self.pub_msn_info.publish(String("Ending Mission {}".format(self.current_mission.name)))
        fprint(str(result) + " TIME: " + str((self.nh.get_time() - self.current_mission.start_time).to_sec()),
               msg_color="green", title="{} MISSION COMPLETE: ".format(self.current_mission.name))
        self.points += self.current_mission.points
        self._mission_complete(self.current_mission)

    @util.cancellableInlineCallbacks
    def _run_mission(self, mission, redo=False):
        if not redo:
            self.pub_msn_info.publish(String("Starting Mission {}".format(mission.name)))
        yield self.nh.sleep(.3)
        self.current_mission = mission
        self.current_mission_defer = self._do_mission(self.current_mission)
        self.current_mission_defer.addCallbacks(self._end_mission, errback=self._err_mission)
        res = yield self.current_mission_defer
        defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def _run_base_mission(self, center_object):
        if self.base_mission is not None:
            base_mission = self._do_mission(self.base_mission, center_object)
            base_mission.addErrback(lambda err: fprint(err, msg_color="red", title="MISSION ERR | BASE MISSION"))
            res = yield base_mission
            defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def _monitor_timeouts(self):
        while len(self.tree) != 0:
            yield self.nh.sleep(.5)
            if self.current_mission is None:
                continue
            # print (self.nh.get_time() - self.current_mission.start_time).to_sec()
            # print self.current_mission.name
            # print self.current_mission.timeout
            if ((self.nh.get_time() - self.current_mission.start_time).to_sec() > self.current_mission.timeout and
                    self.current_mission_defer is not None):
                self.current_mission_defer.cancel()

    @util.cancellableInlineCallbacks
    def empty_queue(self):
        """Constantly empties the queue if there is something in it, or run the base mission otherwise."""
        # TODO CHECK IF ALL MARKERS PLACED
        self.start_time = self.nh.get_time()
        self._monitor_timeouts()
        while len(self.tree) != 0:
            TimeoutManager.generate_timeouts(self._get_time_left(), self.missions_left)
            m = yield self._get_closest_mission()
            yield self._run_mission(m)
            yield self.nh.sleep(1)

        fprint("MISSIONS COMPLETE, TOTAL RUN TIME: {}".format((self.nh.get_time() - self.start_time).to_sec()), msg_color="green")
        fprint("MISSIONS COMPLETE, TOTAL POINTS: {}".format((self.points), msg_color="green"))
