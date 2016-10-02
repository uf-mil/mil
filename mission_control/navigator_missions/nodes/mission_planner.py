#!/usr/bin/env python
from txros import util, NodeHandle
from twisted.internet import defer, reactor
import sys
import rospy
from navigator_singleton.navigator import Navigator
from navigator_msgs.msg import PerceptionObject
import nav_missions
import argparse
import std_msgs

nh = None

class Mission(object):

    def __init__(self, name, item_dep, children):
        self.name = name
        self.item_dep = item_dep
        self.children = children

    @util.cancellableInlineCallbacks
    def do_mission(self):
        print self.name
        n = yield Navigator(nh)._init()
        to_run = getattr(nav_missions, self.name)
        yield to_run.main(n)


class MissionPlanner:

    def __init__(self):
        self.tree = []
        self.queue = []
        self.found = []
        # TODO Put in YAML file
        stc = Mission("scan_the_code", ["scan_the_code"], [])
        bf = Mission("back_and_forth", [], [stc])
        self.tree.append(stc)

    @util.cancellableInlineCallbacks
    def _init(self):
        self.nh = yield NodeHandle.from_argv("mission_planner")
        global nh
        nh = self.nh

        self.sub_database = yield nh.subscribe('/database/object_found', PerceptionObject, self.new_item)
        #self.servcl_exploration_yield = yield nh.get_service_client("/exploration/yield_control", std_msgs.msg.Bool)
        self.refresh()
        defer.returnValue(self)

    def new_item(self, obj):
        self.found.append(obj.name)
        self.refresh()

    def refresh(self):
        for mission in self.tree:
            if(self.can_complete(mission)):
                self.queue.append(mission)
        self.empty_queue()

    def can_complete(self, mission):
        for item in mission.item_dep:
            if item not in self.found:
                return False
        return True

    @util.cancellableInlineCallbacks
    def empty_queue(self):
        if(len(self.queue) == 0):
            return
        for mission in self.queue:
            # add a timeout here
            yield mission.do_mission()
            print "completed mission"
            for child in mission.children:
                self.tree.append(child)
            self.queue.remove(mission)
            self.tree.remove(mission)

        for mission in self.tree:
            if(self.can_complete(mission)):
                self.queue.append(mission)

        self.empty_queue()


@util.cancellableInlineCallbacks
def main():
    om = MissionPlanner()
    od = yield om._init() 

reactor.callWhenRunning(main)
reactor.run()