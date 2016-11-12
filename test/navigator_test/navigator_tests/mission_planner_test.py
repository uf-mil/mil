import abc
from navigator_test_lib import TestUnit, SpoofGenerator
from navigator_msgs.msg import PerceptionObjectArray, PerceptionObject
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryResponse
from nav_missions_lib import MissionPlanner
import yaml
from txros import util


class MissionPlannerTest(TestUnit):

    def __init__(self, nh):
        self.nh = nh

    @util.cancellableInlineCallbacks
    def create_spoofs(self):
        sg = SpoofGenerator()
        arr1 = PerceptionObjectArray()
        arr2 = PerceptionObjectArray()
        arr3 = PerceptionObjectArray()
        obj1 = PerceptionObject()
        obj1.name = "scan_the_code"
        obj2 = PerceptionObject()
        obj2.name = "shooter"
        arr1.objects = []
        arr2.objects = [obj1]
        arr3.objects = [obj1, obj2]
        response1 = ObjectDBQueryResponse()
        response2 = ObjectDBQueryResponse()
        response2.objects = [obj1]
        service = sg.spoof_service("/database/requests", ObjectDBQuery, [response1, response1, response1, response2])
        yield service.start(self.nh)

        self.pub_base_mission = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [arr1, arr2], [5, 1000])
        self.pub_normal_1 = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [arr1, arr2, arr3], [5, 5, 100])
        self.pub_normal_2 = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [arr1, arr2, arr3], [5, 5, 100])
        self.pub_fail_mission = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [], [])
        self.pub_missing_objects = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [arr1, arr2], [5, 10000])

    @util.cancellableInlineCallbacks
    def run_tests(self):
        base_file = '/'.join(__file__.split('/')[0:-1]) + "/mission_yamls"
        yield self._run_mission(base_file + "/mission_fails.yaml", self.pub_fail_mission)
        yield self._run_mission(base_file + "/missing_object.yaml", self.pub_missing_objects)
        yield self._run_mission(base_file + "/base_mission.yaml", self.pub_base_mission)
        yield self._run_mission(base_file + "/normal_behavior_1.yaml", self.pub_normal_1)
        yield self._run_mission(base_file + "/normal_behavior_2.yaml", self.pub_normal_2)

    @util.cancellableInlineCallbacks
    def _run_mission(self, yaml_file, spoof):
        with open(yaml_file, 'r') as stream:
            try:
                spoof.start(self.nh)
                yaml_text = yaml.load(stream)
                init = MissionPlanner(yaml_text, 't').init_()
                planner = yield init
                yield planner.empty_queue()
                yield spoof.stop()
                yield planner.nh.shutdown()
            except yaml.YAMLError as exc:
                print(exc)
