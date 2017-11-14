from navigator_test_lib import TestUnit, SpoofGenerator
from navigator_msgs.msg import PerceptionObjectArray, PerceptionObject
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryResponse
from nav_missions_lib import MissionPlanner
from geometry_msgs.msg import Point
import yaml
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from txros import util
from mil_misc_tools.text_effects import fprint


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

        stc_marker_obj = PerceptionObject()
        stc_marker_obj.name = "Scan_The_Code"
        stc_marker_obj.position = Point(1, 1, 1)
        shooter_marker_obj = PerceptionObject()
        shooter_marker_obj.name = "Shooter"
        shooter_marker_obj.position = Point(2, 2, 2)

        stc_obj = PerceptionObject()
        stc_obj.name = "scan_the_code"
        stc_obj.position = Point(1, 1, 1)
        shooter_obj = PerceptionObject()
        shooter_obj.name = "shooter"
        shooter_obj.position = Point(2, 2, 2)

        empty.objects = []
        stc.objects = [stc_marker_obj]
        stc_shooter.objects = [stc_marker_obj, shooter_marker_obj]

        empty_resp = ObjectDBQueryResponse()
        all_marker = ObjectDBQueryResponse()
        stc_response = ObjectDBQueryResponse()
        shooter_response = ObjectDBQueryResponse()

        all_marker.found = True

        all_marker.objects = [stc_marker_obj, shooter_marker_obj]
        all_marker.found = True

        stc_response.objects = [stc_marker_obj]
        stc_response.found = True

        shooter_response.objects = [shooter_marker_obj]
        shooter_response.found = True

        odom = Odometry()
        odom.pose.pose.position = Point(0, 0, 0)

        self.pub_base_mission = sg.spoof_publisher("/odom", Odometry, [odom], [100000])

        self.serv_empty = sg.spoof_service("/database/requests", ObjectDBQuery, [empty_resp])

        self.serv_markers = sg.spoof_service("/database/requests", ObjectDBQuery, [all_marker])
        self.serv_markers_empty = sg.spoof_service("/database/requests", ObjectDBQuery,
                                                   [empty_resp, all_marker, all_marker, all_marker, all_marker,
                                                    all_marker, all_marker, all_marker, all_marker,
                                                    all_marker, all_marker, all_marker, all_marker,
                                                    all_marker, all_marker, all_marker, all_marker])

        # self.missing_marker = sg.spoof_service("/database/requests", ObjectDBQuery, [stc_response,
        #                                                                                   shooter_response,
        #                                                                                   stc_response,
        #                                                                                   stc_response,
        #                                                                                   shooter_response,
        #                                                                                   shooter_response,
        #                                                                                   stc_response])

        # self.serv_missing_marker = sg.spoof_service("/database/requests", ObjectDBQuery, [empty_resp,
        #                                                                                   stc_response,
        #                                                                                   shooter_response,
        #                                                                                   stc_response,
        #                                                                                   stc_response,
        #                                                                                   shooter_response,
        #                                                                                   shooter_response,
        #                                                                                   stc_response])

        self.markers = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [stc_shooter], [1000])

        self.empty = sg.spoof_publisher("/database/objects", PerceptionObjectArray, [empty], [1000])

    @util.cancellableInlineCallbacks
    def run_tests(self):
        self.pub_base_mission.start(self.nh)
        yield self.nh.subscribe("/mission_planner/mission", String, self.mission_cb)
        base_file = '/'.join(__file__.split('/')[0:-1]) + "/mission_planner_yamls"

        yield self._run_mission(base_file + "/shooter_in_database.yaml", self.empty, self.serv_markers, 30,
                                "shooter is in the database",
                                [("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "test_mission_1"),
                                 ("Ending", "test_mission_1")])

        yield self._run_mission(base_file + "/shooter_not_in_database.yaml", self.empty, self.serv_markers, 30,
                                "shooter is not in the database",
                                [("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Failing", "test_mission_1")])

        yield self._run_mission(base_file + "/safe_exit.yaml", self.empty, self.serv_markers, 30,
                                "Mission fails with a safe exit",
                                [("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "safe_exit_mission"),
                                 ("Failing", "safe_exit_mission"),
                                 ("SafeExiting", "safe_exit_mission")])

        yield self._run_mission(base_file + "/timeout_repeat.yaml", self.empty, self.serv_markers, 1.01,
                                "Mission timeouts, can repeat",
                                [("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "test_mission_1"),
                                 ("TimingOut", "test_mission_1"),
                                 ("Retrying", "test_mission_1"),
                                 ("Starting", "test_mission_1"),
                                 ("Ending", "test_mission_1")])

        yield self._run_mission(base_file + "/normal_behavior_s1.yaml", self.empty, self.serv_markers, 30,
                                "Normal Behavior, two children have the same parent",
                                [("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "test_mission_1"),
                                 ("Ending", "test_mission_1"),
                                 ("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "test_mission_2"),
                                 ("Ending", "test_mission_2"),
                                 ("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "test_mission_3"),
                                 ("Ending", "test_mission_3")])

        yield self._run_mission(base_file + "/missing_marker_s2.yaml", self.empty, self.serv_markers_empty, 30,
                                "Normal Behavior, Missing Marker, one child has the two parents",
                                [("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "test_mission_2"),
                                 ("Ending", "test_mission_2"),
                                 ("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
                                 ("Starting", "test_mission_3"),
                                 ("Ending", "test_mission_3"),
                                 ("Starting", "base_mission"),
                                 ("Ending", "base_mission"),
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
