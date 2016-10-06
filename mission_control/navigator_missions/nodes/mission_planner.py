#!/usr/bin/env python
from txros import util, NodeHandle
from twisted.internet import defer, reactor
from navigator_singleton.navigator import Navigator
from navigator_msgs.msg import PerceptionObject
import nav_missions

nh = None
n = None


class Mission(object):

    def __init__(self, name, item_dep, children):
        self.name = name
        self.item_dep = item_dep
        self.children = children

    @util.cancellableInlineCallbacks
    def do_mission(self):
        print "starting mission:", self.name
        to_run = getattr(nav_missions, self.name)
        yield to_run.main(n)


class MissionPlanner:

    def __init__(self):
        self.tree = []
        self.queue = []  # Make this asynchronous
        self.found = []
        self.completeing_mission = False
        # TODO Put in YAML file
        stc = Mission("scan_the_code", ["scan_the_code"], [])
        bf = Mission("back_and_forth", [], [stc])
        self.tree.append(bf)

    @util.cancellableInlineCallbacks
    def _init(self):
        self.nh = yield NodeHandle.from_argv("mission_planner")
        global nh
        nh = self.nh
        global n
        n = yield Navigator(nh)._init()

        self.sub_database = yield nh.subscribe('/database/object_found', PerceptionObject, self.new_item)
        # self.servcl_exploration_yield = yield nh.get_service_client("/exploration/yield_control", std_msgs.msg.Bool)
        self.refresh()
        defer.returnValue(self)

    def new_item(self, obj):
        self.found.append(obj.name)
        self.refresh()

    def refresh(self):
        for mission in self.tree:
            if(self.can_complete(mission) and mission not in self.queue):
                print "adding mission:", mission.name
                self.queue.append(mission)
        if not self.completeing_mission:
            self.empty_queue()

    def can_complete(self, mission):
        for item in mission.item_dep:
            if item not in self.found:
                return False
        return True

    @util.cancellableInlineCallbacks
    def empty_queue(self):
        self.completeing_mission = True
        if(len(self.queue) == 0):
            self.completeing_mission = False
            return
        for mission in self.queue:
            # add a timeout here
            yield mission.do_mission()
            print "completed mission:", mission.name
            for child in mission.children:
                self.tree.append(child)
            self.queue.remove(mission)
            self.tree.remove(mission)

        for mission in self.tree:
            if(self.can_complete(mission)):
                print "adding mission:", mission.name
                self.queue.append(mission)

        self.empty_queue()


@util.cancellableInlineCallbacks
def main():
    om = MissionPlanner()
    od = yield om._init()

reactor.callWhenRunning(main)
reactor.run()
