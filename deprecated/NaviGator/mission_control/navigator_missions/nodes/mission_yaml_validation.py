#!/usr/bin/env python
import yaml
import os
import nav_missions
from graphviz import Digraph
from navigator_msgs.msg import PerceptionObject


class MissionYAMLValidation(object):

    def __init__(self, yaml_text):
        self.mission_names = [mission_name for mission_name in dir(nav_missions) if mission_name[0] != '_']
        # self.object_names = [PerceptionObject.DETECT_DELIVER_PLATFORM,
        #                      PerceptionObject.IDENTIFY_AND_DOCK,
        #                      PerceptionObject.SCAN_THE_CODE,
        #                      PerceptionObject.TOTEM,
        #                      PerceptionObject.START_GATE_BUOY,
        #                      PerceptionObject.BUOY,
        #                      PerceptionObject.FAKE_SHOOTER,
        #                      PerceptionObject.FAKE_IDENTIFY_AND_DOCK,
        #                      PerceptionObject.FAKE_SCAN_THE_CODE,
        #                      PerceptionObject.GATE1,
        #                      PerceptionObject.GATE2,
        #                      PerceptionObject.GATE3,
        #                      PerceptionObject.FAKE_BUOY_FIELD,
        #                      PerceptionObject.EMPTY_SPACE
        #                      ]
        self.dot = Digraph(comment='The Round Table')
        self._load_missions(yaml_text)

    def _load_missions(self, yaml_text):
        mission_to_id = {}

        for i, name in enumerate(yaml_text.keys()):
            mission = yaml_text[name]
            obj_dep = mission["marker_dep"]
            mis_dep = mission["mission_dep"]
            is_base = mission["is_base"]
            # timeout = mission["timeout"]
            print name
            # assert set(obj_dep).issubset(self.object_names)
            assert name in self.mission_names
            assert mis_dep in yaml_text.keys() or mis_dep == "None"
            assert (is_base is True or is_base is False)
            # assert (timeout == "inf" or timeout is int)
            mission_to_id[name] = str(i)
            print i
            self.dot.node(str(i), "{}\n{}\n{}\n{}".format(name, obj_dep, is_base))
        for i, name in enumerate(yaml_text.keys()):
            mission = yaml_text[name]
            mis_dep = mission["mission_dep"]
            if mis_dep == "None":
                continue
            hash1 = mission_to_id[name]
            hash2 = mission_to_id[mis_dep]
            self.dot.edge(hash2, hash1)

        dir_path = os.path.dirname(os.path.realpath(__file__))
        gv_file = dir_path + "/graph_validations/Missions.gv"
        self.dot.render(gv_file)

if __name__ == "__main__":
    yaml_text = None
    yaml_file = "missions.yaml"
    dir_path = os.path.dirname(os.path.realpath(__file__))
    yaml_file = dir_path + "/" + yaml_file
    print "starting"
    with open(yaml_file, 'r') as stream:
        try:
            yaml_text = yaml.load(stream)
            yaml_validation = MissionYAMLValidation(yaml_text)
        except yaml.YAMLError as exc:
            print(exc)
