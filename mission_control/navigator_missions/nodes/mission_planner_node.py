#!/usr/bin/env python
import os
from txros import util
import yaml
from nav_missions_lib import MissionPlanner
from twisted.internet import reactor
import sys


@util.cancellableInlineCallbacks
def main():
    """The main method."""
    yaml_text = None
    yaml_file = "missions.yaml"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    yaml_file = dir_path + "/" + yaml_file
    sim = False
    if len(sys.argv) > 1 and sys.argv[1] == '-s':
        sim = True
    print sys.argv
    with open(yaml_file, 'r') as stream:
        try:
            yaml_text = yaml.load(stream)
            planner = yield MissionPlanner().init_(yaml_text, sim_mode=sim)
            yield planner.empty_queue()
            reactor.stop()
        except yaml.YAMLError as exc:
            print(exc)


reactor.callWhenRunning(main)
reactor.run()
