import numpy as np
import rospy
import tf
from time import time
from sub8_ros_tools import rosmsg_to_numpy, normalize
from sub8_sim_tools.physics.physics import Box
from sub8_simulation.srv import SimSetPose, SimSetPoseResponse
from sub8_msgs.msg import Thrust, ThrusterCmd
import geometry_msgs.msg as geometry
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class Sub8(Box):
    _linear_damping_coeff = -50  # TODO: Estimate area
    _rotational_damping_coeff = -0.5  # TODO: Estimate area
    _cmd_timeout = 2.
    def __init__(self, world, space, position):
        '''Yes, right now we're approximating the sub as a box
        Not taking any parameters because this is the literal Sub8
        TODO:
            - Make the thruster list a parameter
            - Stop the sub from being able to fly (Only thrust if thruster_z < 0)

        See Annie for how the thrusters work
        '''
        lx, ly, lz = 0.5588, 0.5588, 0.381  # Meters
        self.radius = max((lx, ly, lz)) * 0.5  # For spherical approximation
        volume = lx * ly * lz * 0.25
        # weight = 32.75  # kg
        weight = 25.0  # kg
        density = weight / volume
        super(self.__class__, self).__init__(world, space, position, density, lx, ly, lz)

        self.truth_odom_pub = rospy.Publisher('truth/odom', Odometry, queue_size=1)
        self.thruster_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=2)
        self.thrust_dict = {}

        self.last_force = (0.0, 0.0, 0.0)

        # Make this a parameter
        # name, relative direction, relative position (COM)
        self.thruster_list = [
            ("FLV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, 0.16900, 0.0142])),  # flake8: noqa
            ("FLL", np.array([-0.866,  0.5,  0]), np.array([ 0.2678, 0.27950, 0.0000])),  # flake8: noqa
            ("FRV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, -0.1690, 0.0142])),  # flake8: noqa
            ("FRL", np.array([-0.866, -0.5,  0]), np.array([ 0.2678, -0.2795, 0.0000])),  # flake8: noqa
            ("BLV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, 0.16900, 0.0142])),  # flake8: noqa
            ("BLL", np.array([ 0.866,  0.5,  0]), np.array([-0.2678, 0.27950, 0.0000])),  # flake8: noqa
            ("BRV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, -0.1690, 0.0142])),  # flake8: noqa
            ("BRL", np.array([ 0.866, -0.5,  0]), np.array([-0.2678, -0.2795, 0.0000])),  # flake8: noqa
        ]
        self.last_cmd_time = time()
        self.set_up_ros()

    def set_up_ros(self):
        '''TODO:
            Add pointAt
        '''
        self.position_server = rospy.Service('/sim/vehicle/set_pose', SimSetPose, self.set_pose_server)

    def set_pose_server(self, srv):
        '''Set the pose of the submarine
        TODO: Make this an 'abstract' method of Entity, and assign each new Entity a name/id
        '''
        rospy.logwarn("Manually setting position of simulated vehicle")
        position = rosmsg_to_numpy(srv.pose.position)
        self.body.setPosition(position)

        rotation_q = rosmsg_to_numpy(srv.pose.orientation)
        rotation_norm = np.linalg.norm(rotation_q)

        velocity = rosmsg_to_numpy(srv.twist.linear)
        angular = rosmsg_to_numpy(srv.twist.angular)

        self.body.setLinearVel(velocity)
        self.body.setAngularVel(angular)

        # If the commanded rotation is valid
        if np.isclose(rotation_norm, 1., atol=1e-2):
            self.body.setQuaternion(normalize(rotation_q))
        else:
            rospy.logwarn("Commanded quaternion was not a unit quaternion, NOT setting rotation")

        return SimSetPoseResponse()

    def publish_pose(self):
        '''TODO:
            Publish velocity in body frame
        '''
        pose_matrix = np.transpose(self.pose)
        linear_vel = self.velocity
        angular_vel = self.angular_vel
        quaternion = tf.transformations.quaternion_from_matrix(pose_matrix)
        translation = tf.transformations.translation_from_matrix(pose_matrix)

        header = Header(
            stamp=rospy.Time.now(),
            frame_id='/world'
        )
        pose = geometry.Pose(
            position=geometry.Point(*translation),
            orientation=geometry.Quaternion(*quaternion),
        )

        twist = geometry.Twist(
            linear=geometry.Vector3(*linear_vel),
            angular=geometry.Vector3(*angular_vel)
        )

        odom_msg = Odometry(
            header=header,
            child_frame_id='/body',
            pose=geometry.PoseWithCovariance(
                pose=pose
            ),
            twist=geometry.TwistWithCovariance(
                twist=twist
            )
        )

        self.truth_odom_pub.publish(odom_msg)

    def thrust_cb(self, msg):
        '''TODO: Clamp'''
        self.last_cmd_time = time()
        self.thrust_dict = {}
        for thrust_cmd in msg.thruster_commands:
            self.thrust_dict[thrust_cmd.name] = thrust_cmd.thrust

    def step(self, dt):
        '''Ignore dt
            TODO: Check if thruster is underwater
        '''
        # If timeout, reset thrust commands
        if (time() - self.last_cmd_time) > self._cmd_timeout:
            self.thrust_dict = {}

        if self.pos[2] < 0:
            # Apply thruster force only if underwater

            for i, (name, rel_dir, rel_pos) in enumerate(self.thruster_list):
                thruster_force = self.thrust_dict.get(name, 0.0)
                body_force = rel_dir * thruster_force
                self.body.addRelForceAtRelPos(body_force, rel_pos)

        self.last_force = self.body.getForce()

        self.apply_buoyancy_force()
        self.apply_damping_force()
        # self.apply_damping_torque()
        self.publish_pose()