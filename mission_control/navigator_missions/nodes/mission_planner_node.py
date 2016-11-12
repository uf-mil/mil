#!/usr/bin/env python
import os
from txros import util
import yaml
from nav_missions_lib import MissionPlanner
from twisted.internet import reactor


@util.cancellableInlineCallbacks
def main():
    """The main method."""
    yaml_text = None
    yaml_file = "missions.yaml"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    yaml_file = dir_path + "/" + yaml_file
    print "starting"
    with open(yaml_file, 'r') as stream:
        try:
            yaml_text = yaml.load(stream)
            planner = yield MissionPlanner(yaml_text).init_()
            planner.empty_queue()
        except yaml.YAMLError as exc:
            print(exc)


reactor.callWhenRunning(main)
reactor.run()
