#!/usr/bin/env python
from __future__ import division
import os
import numpy as np
import yaml
import genpy
import rospkg
from twisted.internet import defer
from txros import action, util, tf, NodeHandle
from pose_editor import PoseEditor2
import mil_tools
from ros_alarms import TxAlarmListener
from navigator_path_planner.msg import MoveAction, MoveGoal
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolRequest
from geometry_msgs.msg import PoseStamped, PointStamped
import navigator_msgs.srv as navigator_srvs
from topic_tools.srv import MuxSelect, MuxSelectRequest
from mil_misc_tools.text_effects import fprint
from navigator_tools import MissingPerceptionObject
from mil_tasks_core import BaseTask


class MissionResult(object):
    NoResponse = 0
    ParamNotFound = 1
    DbObjectNotFound = 2
    OtherResponse = 100

    def __init__(self, success=True, response=None, message="", need_rerun=False, post_function=None):
        self.success = success
        self.response = self.NoResponse if response is None else response
        self.message = message
        self.need_rerun = need_rerun
        self.post_function = lambda: None if post_function is None else post_function

    def __repr__(self):
        cool_bars = "=" * 75
        _pass = (cool_bars,
                 "    Mission Success!",
                 "    Message: {}".format(self.message),
                 cool_bars)
        _fail = (cool_bars,
                 "    Mission Failure!",
                 "    Message: {}".format(self.message),
                 "    Post function: {}".format(self.post_function.__name__),
                 cool_bars)

        return '\n'.join(_pass if self.success else _fail)


class Navigator(BaseTask):
    circle = "CIRCLE"
    cross = "CROSS"
    triangle = "TRIANGLE"
    red = "RED"
    green = "GREEN"
    blue = "BLUE"

    def __init__(self, **kwargs):
        super(Navigator, self).__init__(**kwargs)

    @classmethod
    @util.cancellableInlineCallbacks
    def _init(cls, task_runner):
        super(Navigator, cls)._init(task_runner)
        cls.vision_proxies = {}
        cls._load_vision_services()

        cls.mission_params = {}
        cls._load_mission_params()

        # If you don't want to use txros
        cls.pose = None
        cls.ecef_pose = None

        cls.enu_bounds = None

        cls.killed = '?'
        cls.odom_loss = '?'

        if (yield cls.nh.has_param('/is_simulation')):
            cls.sim = yield cls.nh.get_param('/is_simulation')
        else:
            cls.sim = False

        # For missions to access clicked points / poses
        cls.rviz_goal = cls.nh.subscribe("/move_base_simple/goal", PoseStamped)
        cls.rviz_point = cls.nh.subscribe("/clicked_point", PointStamped)

        cls._moveto_client = action.ActionClient(cls.nh, 'move_to', MoveAction)

        def odom_set(odom):
            return setattr(cls, 'pose', mil_tools.odometry_to_numpy(odom)[0])
        cls._odom_sub = cls.nh.subscribe('odom', Odometry, odom_set)

        def enu_odom_set(odom):
            return setattr(cls, 'ecef_pose', mil_tools.odometry_to_numpy(odom)[0])
        cls._ecef_odom_sub = cls.nh.subscribe('absodom', Odometry, enu_odom_set)

        try:
            cls._database_query = cls.nh.get_service_client('/database/requests', navigator_srvs.ObjectDBQuery)
            cls._camera_database_query = cls.nh.get_service_client(
                '/camera_database/requests', navigator_srvs.CameraDBQuery)
            cls._change_wrench = cls.nh.get_service_client('/wrench/select', MuxSelect)
            cls._change_trajectory = cls.nh.get_service_client('/trajectory/select', MuxSelect)
        except AttributeError, err:
            fprint("Error getting service clients in nav singleton init: {}".format(
                err), title="NAVIGATOR", msg_color='red')

        cls.tf_listener = tf.TransformListener(cls.nh)

        yield cls._make_alarms()

        if cls.sim:
            fprint("Sim mode active!", title="NAVIGATOR")
            yield cls.nh.sleep(.5)
        else:
            # We want to make sure odom is working before we continue
            fprint("Action client do you yield?", title="NAVIGATOR")
            yield util.wrap_time_notice(cls._moveto_client.wait_for_server(), 2, "Lqrrt action server")
            fprint("Yes he yields!", title="NAVIGATOR")

            fprint("Waiting for odom...", title="NAVIGATOR")
            odom = util.wrap_time_notice(cls._odom_sub.get_next_message(), 2, "Odom listener")
            enu_odom = util.wrap_time_notice(cls._ecef_odom_sub.get_next_message(), 2, "ENU Odom listener")
            # bounds = util.wrap_time_notice(cls._make_bounds(), 2, "Bounds creation")
            yield defer.gatherResults([odom, enu_odom])  # Wait for all those to finish

    @property
    @util.cancellableInlineCallbacks
    def tx_pose(self):
        last_odom_msg = yield self._odom_sub.get_next_message()
        defer.returnValue(mil_tools.odometry_to_numpy(last_odom_msg)[0])

    @property
    @util.cancellableInlineCallbacks
    def tx_ecef_pose(self):
        last_odom_msg = yield self._ecef_odom_sub.get_next_message()
        defer.returnValue(mil_tools.odometry_to_numpy(last_odom_msg)[0])

    @property
    def move(self):
        return PoseEditor2(self, self.pose)

    def hold(self):
        goal = MoveGoal(move_type='hold')
        return self._moveto_client.send_goal(goal)

    def fetch_result(self, *args, **kwargs):
        # For a unified result class
        return MissionResult(*args, **kwargs)

    @classmethod
    @util.cancellableInlineCallbacks
    def _make_bounds(cls):
        fprint("Constructing bounds.", title="NAVIGATOR")

        if (yield cls.nh.has_param("/bounds/enforce")):
            _bounds = cls.nh.get_service_client('/get_bounds', navigator_srvs.Bounds)
            yield _bounds.wait_for_service()
            resp = yield _bounds(navigator_srvs.BoundsRequest())
            if resp.enforce:
                cls.enu_bounds = [mil_tools.rosmsg_to_numpy(bound) for bound in resp.bounds]
        else:
            fprint("No bounds param found, defaulting to none.", title="NAVIGATOR")
            cls.enu_bounds = None

    @util.cancellableInlineCallbacks
    def database_query(self, object_name=None, raise_exception=True, **kwargs):
        if object_name is not None:
            kwargs['name'] = object_name
            res = yield self._database_query(navigator_srvs.ObjectDBQueryRequest(**kwargs))

            if not res.found and raise_exception:
                raise MissingPerceptionObject(kwargs['name'])

            defer.returnValue(res)

        res = yield self._database_query(navigator_srvs.ObjectDBQueryRequest(**kwargs))
        defer.returnValue(res)

    @util.cancellableInlineCallbacks
    def camera_database_query(self, object_name=None, **kwargs):
        if object_name is not None:
            kwargs['name'] = object_name
            res = yield self._camera_database_query(navigator_srvs.CameraDBQueryRequest(**kwargs))

            defer.returnValue(res)

        res = yield self._database_query(navigator_srvs.CameraDBQueryRequest(**kwargs))
        defer.returnValue(res)

    def vision_request(self, request_name, **kwargs):
        fprint("DEPRECATED: Please use new dictionary based system.")
        return self.vision_proxies[request_name].get_response(**kwargs)

    @classmethod
    def change_wrench(cls, source):
        return cls._change_wrench(MuxSelectRequest(source))

    @classmethod
    def change_trajectory(cls, source):
        return cls._change_trajectory(MuxSelectRequest(source))

    def search(self, *args, **kwargs):
        return Searcher(self, *args, **kwargs)

    @util.cancellableInlineCallbacks
    def latching_publisher(self, topic, msg_type, msg, freq=2, update_header=True):
        '''
        Creates a publisher that publishes a message at a set frequency.
        Usage:
            latched = navigator.latching_publisher("/ogrid", OccupancyGrid, my_ogrid)
            # Now the ogrid is publishing twice a second...
            # If you want to cancel the publisher:
            latched.cancel()
        '''
        pub = yield self.nh.advertise(topic, msg_type)

        while True:
            # Update timestamps if the msg has a header
            if update_header and hasattr(msg, "header"):
                msg.header.stamp = self.nh.get_time()

            pub.publish(msg)
            yield self.nh.sleep(1.0 / freq)

    @classmethod
    def _load_vision_services(cls, fname="vision_services.yaml"):
        rospack = rospkg.RosPack()
        config_file = os.path.join(rospack.get_path('navigator_missions'), 'launch', fname)
        f = yaml.load(open(config_file, 'r'))

        for name in f:
            try:
                s_type = getattr(navigator_srvs, f[name]["type"])
                s_req = getattr(navigator_srvs, "{}Request".format(f[name]["type"]))
                s_args = f[name]['args']
                s_client = cls.nh.get_service_client(f[name]["topic"], s_type)
                s_switch = cls.nh.get_service_client(f[name]["topic"] + '/switch', SetBool)
                cls.vision_proxies[name] = VisionProxy(s_client, s_req, s_args, s_switch)
            except Exception, e:
                err = "Error loading vision sevices: {}".format(e)
                fprint("" + err, title="NAVIGATOR", msg_color='red')

    @classmethod
    def _load_mission_params(cls, fname="mission_params.yaml"):
        rospack = rospkg.RosPack()
        config_file = os.path.join(rospack.get_path('navigator_missions'), 'launch', fname)
        f = yaml.load(open(config_file, 'r'))

        for name in f:
            try:
                param = f[name]["param"]
                options = f[name]["options"]
                desc = f[name]["description"]
                default = f[name].get("default")
                cls.mission_params[name] = MissionParam(cls.nh, param, options, desc, default)
            except Exception, e:
                err = "Error loading mission params: {}".format(e)
                fprint("" + err, title="NAVIGATOR", msg_color='red')

    @classmethod
    def kill_alarm_cb(cls, _, alarm):
        cls.killed = alarm.raised
        cls.kill_alarm = alarm

    @classmethod
    @util.cancellableInlineCallbacks
    def _make_alarms(cls):
        cls.kill_listener = yield TxAlarmListener.init(cls.nh, 'kill', cls.kill_alarm_cb)
        cls.odom_loss_listener = yield TxAlarmListener.init(
            cls.nh, 'odom-kill',
            lambda _, alarm: setattr(cls, 'odom_loss', alarm.raised))
        fprint("Alarm listener created, listening to alarms: ", title="NAVIGATOR")

        cls.kill_alarm = yield cls.kill_listener.get_alarm()
        cls.killed = cls.kill_alarm.raised
        cls.odom_loss = yield cls.odom_loss_listener.is_raised()
        fprint("\tkill :", newline=False)
        fprint(cls.killed)
        fprint("\todom-kill :", newline=False)
        fprint(cls.odom_loss)


class VisionProxy(object):

    def __init__(self, client, request, args, switch):
        self.client = client
        self.request = request
        self.args = args
        self.switch = switch

    def start(self):
        # Returns deferred object, make sure to yield on this (same for below)
        return self.switch(SetBoolRequest(data=True))

    def stop(self):
        return self.switch(SetBoolRequest(data=False))

    def get_response(self, **kwargs):
        s_args = dict(self.args.items() + kwargs.items())
        s_req = self.request(**s_args)

        return self.client(s_req)


class MissionParam(object):

    def __init__(self, nh, param, options, desc, default):
        self.nh = nh
        self.param = param
        self.options = options
        self.description = desc
        self.default = default

    @util.cancellableInlineCallbacks
    def get(self, raise_exception=True):
        # Returns deferred object, make sure to yield on this (same for below)
        if not (yield self.exists()):
            if self.default is not None:
                yield self.set(self.default)
                defer.returnValue(self.default)
            if raise_exception:
                raise Exception("Mission Param {} not yet set".format(self.param))
            else:
                defer.returnValue(False)
        value = yield self.nh.get_param(self.param)
        if not self._valid(value):
            raise Exception("Value {} is invalid for param {}\nValid values: {}\nDescription: {}".format(
                value, self.param, self.options, self.description))
        else:
            defer.returnValue(value)

    def exists(self):
        return self.nh.has_param(self.param)

    @util.cancellableInlineCallbacks
    def set(self, value):
        if not self._valid(value):
            raise Exception("Value {} is invalid for param {}\nValid values: {}\nDescription: {}".format(
                value, self.param, self.options, self.description))
        yield self.nh.set_param(self.param, value)

    @util.cancellableInlineCallbacks
    def valid(self):
        exists = yield self.exists()
        if not exists:
            defer.returnValue(False)
        value = yield self.nh.get_param(self.param)
        if not self._valid(value):
            defer.returnValue(False)
        defer.returnValue(True)

    @util.cancellableInlineCallbacks
    def reset(self):
        if (yield self.exists()):
            if self.default is not None:
                yield self.set(self.default)
            else:
                yield self.nh.delete_param(self.param)

    def _valid(self, value):
        for x in self.options:
            if x == value:
                return True
        return False


class Searcher(object):
    def __init__(self, nav, search_pattern=None, looker=None, vision_proxy="test", **kwargs):
        self.nav = nav
        self.looker = looker
        if looker is None:
            self.looker = self._vision_proxy_look
            self.vision_proxy = vision_proxy
        self.looker_kwargs = kwargs
        self.search_pattern = search_pattern

        self.object_found = False
        self.pattern_done = False
        self.response = None
        print "dfdf"

    def catch_error(self, failure):
        if failure.check(defer.CancelledError):
            fprint("Cancelling defer.", title="SEARCHER")
        else:
            fprint("There was an error.", title="SEARCHER", msg_color='red')
            fprint(failure.printTraceback())
            # Handle error

    @util.cancellableInlineCallbacks
    def start_search(self, timeout=120, loop=True, spotings_req=2, **kwargs):
        fprint("Starting.", title="SEARCHER")
        looker = self._run_look(spotings_req).addErrback(self.catch_error)
        finder = self._run_search_pattern(loop, **kwargs).addErrback(self.catch_error)

        start_pose = self.nav.move.forward(0)
        start_time = self.nav.nh.get_time()
        try:
            while self.nav.nh.get_time() - start_time < genpy.Duration(timeout):
                if self.object_found:
                    finder.cancel()
                    fprint("Object found.", title="SEARCHER")
                    defer.returnValue(True)
                if self.pattern_done and not loop:
                    finder.cancel()
                    defer.returnValue(False)
                yield self.nav.nh.sleep(0.1)

        except KeyboardInterrupt:
            # This doesn't work...
            fprint("Control C detected!", title="SEARCHER")

        fprint("Object NOT found. Returning to start position.", title="SEARCHER")
        finder.cancel()
        looker.cancel()

        yield start_pose.go()

    @util.cancellableInlineCallbacks
    def _run_search_pattern(self, loop, **kwargs):
        '''
        Look around using the search pattern.
        If `loop` is true, then keep iterating over the list until timeout is reached or we find it.
        '''
        if self.search_pattern is None:
            return

        def pattern():
            for pose in self.search_pattern:
                fprint("Going to next position.", title="SEARCHER")
                if type(pose) == list or type(pose) == np.ndarray:
                    yield self.nav.move.relative(pose).go(**kwargs)
                else:
                    yield pose.go(**kwargs)

                yield self.nav.nh.sleep(2)
            if not loop:
                fprint("Search Pattern Over", title="SEARCHER")
                self.pattern_done = True

        fprint("Executing search pattern.", title="SEARCHER")

        if loop:
            while True:
                yield util.cancellableInlineCallbacks(pattern)()
        else:
            yield util.cancellableInlineCallbacks(pattern)()

    @util.cancellableInlineCallbacks
    def _vision_proxy_look(self):
        resp = yield self.nav.vision_proxies[self.vision_proxy].get_response(**self.looker_kwargs)
        defer.returnValue(resp.found)

    @util.cancellableInlineCallbacks
    def _run_look(self, spotings_req):
        '''
        Look for the object using the vision proxy.
        Only return true when we spotted the objects `spotings_req` many times (for false positives).
        '''
        spotings = 0
        fprint("Looking for object.", title="SEARCHER")
        while spotings < spotings_req:
            if (yield self.looker(**self.looker_kwargs)):
                fprint("Object found! {}/{}".format(spotings + 1, spotings_req), title="SEARCHER")
                spotings += 1
            else:
                spotings = 0

            yield self.nav.nh.sleep(0.5)

        if spotings >= spotings_req:
            self.object_found = True


@util.cancellableInlineCallbacks
def main():
    nh = yield NodeHandle.from_argv("navigator_singleton")
    n = yield Navigator(nh)._init()
    fprint((yield n.vision_proxies['start_gate'].get_response()))


if __name__ == '__main__':
    from twisted.internet import reactor
    reactor.callWhenRunning(main)
    reactor.run()
