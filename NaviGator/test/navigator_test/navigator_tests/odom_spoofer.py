from axros import NodeHandle
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from navigator_test_lib import SpoofGenerator, TestUnit


class MissionPlannerTest(TestUnit):
    """
    Attributes:
        nh (NodeHandle): it is an object that ensures the proper setup of processes
        results (array): an unused variable that could store a list results for each process
        count (int): a unused variable that could be utilized for incrementing 
    """
    def __init__(self, nh: NodeHandle):
        self.nh = nh
        self.results = []
        self.count = 0

    def create_spoofs(self):
        """
        A Spoof Generator is being created and the mission is being setup to start. 
        """
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
        """
        Puts the object back to sleep after running each and every test case. 
        """
        await self.nh.sleep(10000)
