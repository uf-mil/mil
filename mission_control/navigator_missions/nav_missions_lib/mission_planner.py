#!/usr/bin/env python
"""Mission Planner Module that uses a DAG and YAML to find which mission to perform."""
from txros import util, NodeHandle
from twisted.internet import defer
from navigator_singleton.navigator import Navigator
import nav_missions
import nav_missions_test
from navigator_tools import DBHelper
from std_msgs.msg import String
from mil_misc_tools.text_effects import fprint
from timeout_manager import TimeoutManager
from YAML_parser import yaml_parse
__author__ = "Tess Bianchi"


class MissionPlanner:
    """The class that plans which mission to do next."""

    def __init__(self, mode='r', total_minutes=30):
        """Initialize the MissionPlanner class."""
        self.tree = []
        self.total_time_save = total_minutes * 60
        self.total_time = total_minutes * 60
        self.points = 0
        self.missions_left = []
        self.failed = False

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
        """Initialize the txros aspects of the MissionPlanner."""
        self.sim_mode = sim_mode
        assert yaml_text is not None, "YOU NEED A YAML TEXT TO RUN A MISSION"
        self.nh = yield NodeHandle.from_argv("mission_planner")
        self.navigator = yield Navigator(self.nh)._init(sim_mode)
        self.missions_left, self.base_mission, self.tree, self.total_time = yield yaml_parse(yaml_text, self.navigator,
                                                                                             self.total_time)
        self.pub_msn_info = yield self.nh.advertise("/mission_planner/mission", String)
        self.helper = yield DBHelper(self.nh).init_(navigator=self.navigator)

        yield self.nh.sleep(1)
        defer.returnValue(self)

# helper methods

    @util.cancellableInlineCallbacks
    def _get_closest_mission(self):
        yield self.nh.sleep(1)
        defer.returnValue(self.tree[0])
        # os = []
        # for m in self.tree:
        #     os.append(m.marker)
        # if len(os) == 0:
        #     defer.returnValue(self.tree[0])
        # closest = yield self.helper.get_closest_object(os)
        # for m in self.tree:
        #     if m.marker.name == closest.name:
        #         defer.returnValue(m)

    def _get_time_left(self):
        return self.total_time - (self.nh.get_time() - self.start_time).to_sec()

    def _get_real_timeleft(self):
        return self.total_time_save - (self.nh.get_time() - self.start_time).to_sec()

    @util.cancellableInlineCallbacks
    def publish(self, action, mission=None):
        """Publish a action taken by the mission control system."""
        if mission is None:
            yield self.pub_msn_info.publish(String("{} : {}".format(action, self.current_mission.name)))
        else:
            yield self.pub_msn_info.publish(String("{} : {}".format(action, mission.name)))
        yield self.nh.sleep(.1)


# Mission beginning

    @util.cancellableInlineCallbacks
    def _do_mission(self, mission, **kwargs):
        """Perform a mission, and ensure that all of the post conditions are enforced."""
        mission.attempts += 1
        res = yield mission.do_mission(self.navigator, self, self.module, **kwargs)
        defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def _run_mission(self, mission, redo=False):
        self.current_mission = mission
        if not redo:
            res = yield self._run_base_mission()
            if not res:
                yield self._err_mission(ValueError())
                defer.returnValue(False)

        self.current_mission.start_time = self.nh.get_time()
        self.current_mission_defer = self._do_mission(self.current_mission)
        self.current_mission_defer.addCallbacks(self._cb_mission, errback=self._err_mission)
        res = yield self.current_mission_defer
        self.failed = False
        defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def _run_base_mission(self):
        if self.base_mission is not None:
            base_mission = self._do_mission(self.base_mission, center_marker=self.current_mission.marker,
                                            looking_for=self.current_mission.looking_for)
            base_mission.addErrback(lambda err: fprint(err, msg_color="red", title="MISSION ERR | BASE MISSION"))
            res = yield base_mission
            yield self.publish("Ending", mission=self.base_mission)
            defer.returnValue(res)

# Ending mission
    def _mission_complete(self):
        mission = self.current_mission
        self.current_mission = None
        self.tree.remove(mission)
        for m in mission.children:
            m.mission_deps_left -= 1
            if m.mission_deps_left <= 0:
                self.tree.append(m)
        self.missions_left.remove(mission)

# Callbacks

    @util.cancellableInlineCallbacks
    def _err_mission(self, err):
        if hasattr(err, "type") and err.type == defer.CancelledError:  # This means there was a timeout
            fprint(self.current_mission.name, msg_color="red", title="MISSION TIMEOUT")
            yield self.publish("TimingOut")
            if (TimeoutManager.can_repeat(self.missions_left, self._get_time_left(), self.current_mission)) or len(self.missions_left) == 1:
                yield self.publish("Retrying")
                self.failed = False
                self.current_mission.timeout = self.current_mission.min_time
                yield self._run_mission(self.current_mission, redo=True)
                defer.returnValue(False)
        else:
            fprint(err, msg_color="red", title="{} MISSION ERROR: ".format(self.current_mission.name))
        yield self.publish("Failing")
        yield self.current_mission.safe_exit(self.navigator, err, self, self.module)
        self._mission_complete()

    @util.cancellableInlineCallbacks
    def _cb_mission(self, result):
        if not self.failed:
            yield self.publish("Ending")
            fprint(str(result) + " TIME: " + str((self.nh.get_time() - self.current_mission.start_time).to_sec()),
                   msg_color="green", title="{} MISSION COMPLETE: ".format(self.current_mission.name))
            self.points += self.current_mission.points
            self._mission_complete()

    @util.cancellableInlineCallbacks
    def _monitor_timeouts(self):
        while len(self.tree) != 0:
            yield self.nh.sleep(.1)
            if self.current_mission is None or self.current_mission.start_time is None or self.current_mission.timeout is None:
                continue
            # print (self.nh.get_time() - self.current_mission.start_time).to_sec()
            # print self.current_mission.name
            # print self.current_mission.timeout

            if ((self.nh.get_time() - self.current_mission.start_time).to_sec() > self.current_mission.timeout and
                    self.current_mission_defer is not None):
                if not self.current_mission_defer.called:
                    self.current_mission_defer.cancel()
                self.current_mission_defer = None

    @util.cancellableInlineCallbacks
    def empty_queue(self):
        """Constantly empties the queue if there is something in it, or run the base mission otherwise."""
        # TODO CHECK IF ALL MARKERS PLACED
        self.start_time = self.nh.get_time()
        self._monitor_timeouts()
        while len(self.tree) != 0:
            time_left = self._get_time_left()
            real_time_left = self._get_real_timeleft()
            if real_time_left < 0:
                break
            TimeoutManager.generate_timeouts(time_left, real_time_left, self.missions_left)
            m = yield self._get_closest_mission()
            print m.name
            yield self._run_mission(m)

        fprint("MISSIONS COMPLETE, TOTAL RUN TIME: {}".format((self.nh.get_time() - self.start_time).to_sec()), msg_color="green")
        fprint("MISSIONS COMPLETE, TOTAL POINTS: {}".format((self.points), msg_color="green"))
