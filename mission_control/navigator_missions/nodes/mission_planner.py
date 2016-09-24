#!/usr/bin/env python
from txros import util, NodeHandle
from twisted.internet import defer, reactor
import sys
import rospy

from navigator_singleton.navigator import Navigator
from navigator_msgs.msg import PerceptionObject

import nav_missions

import argparse

@util.cancellableInlineCallbacks
def go(mission):
    nh, args = yield NodeHandle.from_argv_with_remaining(mission.name)
    n = yield Navigator(nh)._init()
    to_run = getattr(nav_missions, mission.name)
    print to_run
    yield to_run.main(n)
    "done"
    defer.returnValue(reactor.stop())

class Mission(object):
    def __init__(self,name,item_dep,children):
        self.name = name
        self.item_dep = item_dep
        self.children = children

    # def poop(self):
    #   yield print "hi"
    @util.cancellableInlineCallbacks
    def hi(self):
        yield go(self)

    def do_mission(self):
        reactor.callWhenRunning(self.hi)
        reactor.run()

class MissionPlanner:

    def __init__(self):
        self.tree = []
        self.queue = []
        self.found = []
        # Put in YAML file
        stc = Mission("scan_the_code", ["scan_the_code"], [])
        bf = Mission("back_and_forth", [], [stc])
        self.tree.append(bf)
        self._odom_sub = rospy.Subscriber('/vision/object_classifier', PerceptionObject, self.new_item)
        self.refresh()

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

    def empty_queue(self):
        if(len(self.queue) == 0):
            return
        for mission in self.queue:
            # add a timeout here 
            mission.do_mission()
            print "sup"
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
    mp = MissionPlanner()
    rospy.init_node('navigator_mission_planner', anonymous=True)
    print "hgy"
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()

