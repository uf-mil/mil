import abc
from navigator_test_lib import TestUnit, SpoofGenerator
from navigator_msgs.msg import PerceptionObjectArray, PerceptionObject
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryResponse
from nav_missions_lib import MissionPlanner
from geometry_msgs.msg import Point
import yaml
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from txros import util
from navigator_tools import fprint


class MissionPlannerTest(TestUnit):

    def __init__(self, nh):
        self.nh = nh
        self.results = []
        self.count = 0

    def mission_cb(self, mission):
        status, word, name = mission.data.split()
        self.results.append((status, name))

    def myassert(self, my_res):
        passed = True
        if len(my_res) != len(self.results):
            passed = False
            fprint("EXPECTED RESULT {} DOES NOT MATCH ACTUAL RESULT {}. TEST FAILED".format(my_res, self.results),
                   msg_color="red")
            fprint("-----------\n\n\n\n", msg_color="red")
            return
        for i, result in enumerate(my_res):
            if result[0] != self.results[i][0] or result[1] != self.results[i][1]:
                passed = False
        if not passed:
            fprint("EXPECTED RESULT {} DOES NOT MATCH ACTUAL RESULT {}. TEST FAILED".format(my_res, self.results),
                   msg_color="red")
            fprint("-----------\n\n\n\n", msg_color="red")
        else:
            fprint("CONGRATS! TEST PASSED", msg_color="green")
            fprint("-----------\n\n\n\n", msg_color="green")
        self.results = []
        self.count += 1

    def create_spoofs(self):
        sg = SpoofGenerator()
        empty = PerceptionObjectArray()
        stc = PerceptionObjectArray()
        stc_shooter = PerceptionObjectArray()

        stc_obj = PerceptionObject()
        stc_obj.name = "ScanTheCode"
        stc_obj.position = Point(1, 1, 1)
        shooter_obj = PerceptionObject()
        shooter_obj.name = "Shooter"
        shooter_obj.position = Point(2, 2, 2)

        empty.objects = []
        stc.objects = [stc_obj]
        stc_shooter.objects = [stc_obj, shooter_obj]

        empty_resp = ObjectDBQueryResponse()
        all_marker = ObjectDBQueryResponse()
        stc_response = ObjectDBQueryResponse()
        shooter_response = ObjectDBQueryResponse()

        all_marker.found = True

        all_marker.objects = [stc_obj, shooter_obj]
        all_marker.found = True

        stc_response.objects = [stc_obj]
        stc_response.found = True

        shooter_response.objects = [shooter_obj]
        shooter_response.found = True

        odom = Odometry()
        odom.pose.pose.position = Point(0, 0, 0)

        self.pub_base_mission = sg.spoof_publisher("/odom", Odometry, [odom], [100000])

        self.serv_empty = sg.spoof_service("/database/requests", ObjectDBQuery, [empty_resp])

        self.serv_markers = sg.spoof_service("/database/requests", ObjectDBQuery, [all_marker])

        self.serv_normal_mission = sg.spoof_service("/database/requests", ObjectDBQuery, [stc_response,
                                                                                          shooter_response,
                                                                                          stc_response,
                                                                                          stc_response,
                                                                                          shooter_response,
                                                                                          shooter_response,
                                                                                          stc_response])

        self.serv_missing_marker = sg.spoof_service("/database/requests", ObjectDBQuery, [empty_resp,
                                                                                          stc_response,
                                                                                          shooter_response,
                                                                                          stc_response,
                                                                                          stc_response,
                                                                                          shooter_response,
                                                                                          shooter_response,
                                                                                          stc_response])

        self.markers = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [stc_shooter], [1000])

        self.empty = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [empty], [1000])

    @util.cancellableInlineCallbacks
    def run_tests(self):
        self.pub_base_mission.start(self.nh)
        yield self.nh.subscribe("/mission_planner/mission", String, self.mission_cb)
        base_file = '/'.join(__file__.split('/')[0:-1]) + "/mission_planner_yamls"
        yield self._run_mission(base_file + "/object_dep_not_found.yaml", self.empty, self.serv_empty, 30,
                                "Mission fails with missing perception object > base mission can't find it",
                                [("Starting", "test_missing_perception_object"),
                                 ("Failing", "test_missing_perception_object")])

        yield self._run_mission(base_file + "/object_dep_found.yaml", self.empty, self.serv_empty, 30,
                                "Mission fails with missing perception object > base mission finds it",
                                [("Starting", "test_missing_perception_object"),
                                 ("Retrying", "test_missing_perception_object"),
                                 ("Ending", "test_missing_perception_object")])

        yield self._run_mission(base_file + "/mission_timeout.yaml", self.empty, self.serv_empty, 2.25,
                                "Mission timeouts > No time for repeat",
                                [("Starting", "test_timeout_mission"),
                                 ("Failing", "test_timeout_mission"),
                                 ("Starting", "test_mission"),
                                 ("Ending", "test_mission")])

        yield self._run_mission(base_file + "/normal_behavior.yaml", self.markers, self.serv_normal_mission, 30,
                                "Normal Behavior",
                                [("Starting", "test_mission_1"),
                                 ("Ending", "test_mission_1"),
                                 ("Starting", "test_mission_2"),
                                 ("Ending", "test_mission_2"),
                                 ("Starting", "test_mission_3"),
                                 ("Ending", "test_mission_3")])

        yield self._run_mission(base_file + "/missing_marker.yaml", self.markers, self.serv_missing_marker, 30,
                                "Normal Behavior, Missing Marker",
                                [("Starting", "test_mission_2"),
                                 ("Ending", "test_mission_2"),
                                 ("Starting", "test_mission_3"),
                                 ("Ending", "test_mission_3"),
                                 ("Starting", "test_mission_4"),
                                 ("Ending", "test_mission_4")])

        fprint("{} Missions passed".format(self.count), msg_color="green")

    @util.cancellableInlineCallbacks
    def _run_mission(self, yaml_file, spoof_pub, spoof_service, time, desc, res):
        with open(yaml_file, 'r') as stream:
            try:
                spoof_pub.start(self.nh)
                yield spoof_service.start(self.nh)
                fprint(desc, msg_color='green', title="STARTING TEST")
                yield self.nh.sleep(.5)
                yaml_text = yaml.load(stream)
                init = MissionPlanner(total_minutes=time, mode='t').init_(yaml_text, sim_mode=True)
                planner = yield init
                yield planner.empty_queue()
                yield spoof_pub.stop()
                yield spoof_service.stop()
                yield planner.nh.shutdown()
                self.myassert(res)
            except yaml.YAMLError as exc:
                print(exc)
