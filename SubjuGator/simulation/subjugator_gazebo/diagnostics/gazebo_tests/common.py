from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SetModelState, SetModelStateRequest, SpawnModel
from geometry_msgs.msg import Pose, Vector3
from kill_handling.msg import Kill
from kill_handling.srv import SetKill, SetKillRequest
from mil_ros_tools import numpy_to_twist
from nav_msgs.msg import Odometry


class Job:

    """Inherit from this!"""

    _job_name = "generic job"

    def __init__(self, nh):
        self.nh = nh
        self.true_pose_sub = nh.subscribe("/world_odom", Odometry)
        self.delete_model = nh.get_service_client("/gazebo/delete_model", DeleteModel)
        self.spawn_model = nh.get_service_client("/gazebo/spawn_sdf_model", SpawnModel)
        self.model_states = nh.subscribe("/gazebo/model_states", ModelStates)

    @property
    def true_pose(self):
        """Still have to yield on this!"""
        return self.true_pose_sub.get_next_message()

    async def initial_setup(self):
        """Run this setup once, to prepare"""
        raise NotImplemented

    async def setup(self):
        """Set up for a single test (And undo any shenanigans done by the previous test)"""
        await self.true_pose_sub.setup()
        await self.model_states.setup()

    async def run(self, sub):
        """Run your tests"""
        raise NotImplementedError

    async def set_model_position(self, position, model="sub8"):
        current_pose = await self.true_pose
        pose = Pose(
            position=Vector3(*position), orientation=current_pose.pose.pose.orientation
        )
        await self.set_model_pose(pose, model=model)

    async def set_model_pose(self, pose, twist=None, model="sub8"):
        """
        Set the position of 'model' to 'pose'.
        It may be helpful to kill the sub before moving it.

        TODO:
            - Deprecate kill stuff
        """
        set_state = self.nh.get_service_client("/gazebo/set_model_state", SetModelState)

        if twist is None:
            twist = numpy_to_twist([0, 0, 0], [0, 0, 0])

        model_state = ModelState()
        model_state.model_name = model
        model_state.pose = pose
        model_state.twist = twist

        if model == "sub8":
            # TODO: Deprecate kill stuff (Zach's PR upcoming)
            kill = self.nh.get_service_client("/set_kill", SetKill)
            await kill(SetKillRequest(kill=Kill(id="initial", active=False)))
            await kill(SetKillRequest(kill=Kill(active=True)))
            await self.nh.sleep(0.1)
            await set_state(SetModelStateRequest(model_state))
            await self.nh.sleep(0.1)
            await kill(SetKillRequest(kill=Kill(active=False)))
        else:
            set_state(SetModelStateRequest(model_state))
