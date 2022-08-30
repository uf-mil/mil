from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from navigator_test_lib import SpoofGenerator, TestUnit
from txros import NodeHandle, util


class MissionPlannerTest(TestUnit):
    def __init__(self, nh: NodeHandle):
        self.nh = nh
        self.results = []
        self.count = 0

    def create_spoofs(self):
        sg = SpoofGenerator()
        odom = Odometry()
        odom.pose.pose.position = Point(0, 0, 0)
        self.pub_base_mission = sg.spoof_publisher("/odom", Odometry, [odom], [100000])
        self.pub_base_mission.start(self.nh)

        self.pub_base_mission1 = sg.spoof_publisher(
            "/absodom", Odometry, [odom], [100000]
        )
        self.pub_base_mission1.start(self.nh)

    async def run_tests(self):
        await self.nh.sleep(10000)
