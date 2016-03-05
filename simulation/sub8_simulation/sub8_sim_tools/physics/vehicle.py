import numpy as np
import rospy
import sub8_ros_tools as sub8_utils
from sub8_sim_tools.physics.physics import Entity
from sub8_simulation.srv import SimSetPose, SimSetPoseResponse
from sub8_msgs.msg import Thrust, VelocityMeasurement, VelocityMeasurements
import geometry_msgs.msg as geometry
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import ode


class Sub8(Entity):
    _linear_damping_coeff = -250  # TODO: Estimate area
    _rotational_damping_coeff = -0.5  # TODO: Estimate area
    _cmd_timeout = rospy.Duration(2.)

    def __init__(self, world, space, position):
        '''Yes, right now we're approximating the sub as a box
        Not taking many parameters because this is the literal Sub8

        thruster_layout: A list of tuples, each tuple formed as...
                (thruster_name, relative_direction, position_wrt_sub_center-of-mass)
                All of these in the FLU body frame

        TODO:
            - Independent publishing rates

        See Annie for how the thrusters work
        '''
        lx, ly, lz = 0.5588, 0.5588, 0.381  # Meters
        self.radius = max((lx, ly, lz)) * 0.365  # For spherical approximation
        self.mass = 32.75  # kg
        # super(self.__class__, self).__init__(world, space, position, density, lx, ly, lz)

        self.body = ode.Body(world)
        self.body.setPosition(position)
        M = ode.Mass()
        M.setBoxTotal(self.mass, lx, ly, lz)
        self.body.setMass(M)
        self.geom = ode.GeomBox(space, lengths=(lx, ly, lz))
        self.geom.setBody(self.body)

        self.truth_odom_pub = rospy.Publisher('truth/odom', Odometry, queue_size=1)
        self.imu_sensor_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.dvl_sensor_pub = rospy.Publisher('dvl', VelocityMeasurements, queue_size=1)
        self.thruster_sub = rospy.Subscriber('thrusters/thrust', Thrust, self.thrust_cb, queue_size=2)
        self.thrust_dict = {}

        self.last_force = (0.0, 0.0, 0.0)
        self.last_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self.cur_vel = np.array([0.0, 0.0, 0.0], dtype=np.float32)

        self.space = space
        self.g = np.array(world.getGravity())

        # TODO: Fix position of DVL, IMU
        # We assume that the DVL and IMU are xyz-FLU aligned (No rotation w.r.t sub)
        self.dvl_position = np.array([0.0, 0.0, 0.0])
        self.imu_position = np.array([0.0, 0.0, 0.0])

        # Define 4 rays to implement a DVL sensor and the relative position of the sensor on the sub
        self.dvl_ray_geoms = (
            ode.GeomRay(None, 1e3),
            ode.GeomRay(None, 1e3),
            ode.GeomRay(None, 1e3),
            ode.GeomRay(None, 1e3),
        )

        self.dvl_ray_orientations = (
            np.array([+0.866, -.5, -1]),
            np.array([+0.866, +.5, -1]),
            np.array([-0.866, +.5, -1]),
            np.array([-0.866, -.5, -1])
        )

        self.dvl_rays = zip(self.dvl_ray_geoms, self.dvl_ray_orientations)

        # Make this a parameter
        # name, relative direction, relative position (COM)
        self.thruster_list = [
            ("FLV", np.array([+0.000, +0.0, -1.]), np.array([+0.1583, +0.1690, +0.0142])),
            ("FLL", np.array([-0.866, +0.5, +0.]), np.array([+0.2678, +0.2795, +0.0000])),
            ("FRV", np.array([+0.000, +0.0, -1.]), np.array([+0.1583, -0.1690, +0.0142])),
            ("FRL", np.array([-0.866, -0.5, +0.]), np.array([+0.2678, -0.2795, +0.0000])),
            ("BLV", np.array([+0.000, +0.0, +1.]), np.array([-0.1583, +0.1690, +0.0142])),
            ("BLL", np.array([+0.866, +0.5, +0.]), np.array([-0.2678, +0.2795, +0.0000])),
            ("BRV", np.array([+0.000, +0.0, +1.]), np.array([-0.1583, -0.1690, +0.0142])),
            ("BRL", np.array([+0.866, -0.5, +0.]), np.array([-0.2678, -0.2795, +0.0000])),
        ]
        self.thrust_range = (-70, 100)
        self.last_cmd_time = rospy.Time.now()
        self.set_up_ros()

    def keypress_callback(self, data):
        """Allows you to manipulate the vehicle through keypress
        TODO: move this into the widget
        """
        keypress = data.data

        # Map keys to forces
        forces = {
            'j': np.array([-500., 0.0, 0.0]),
            'l': np.array([500., 0.0, 0.0]),
            'i': np.array([0.0, 500., 0.0]),
            'k': np.array([0.0, -500., 0.0]),
            'u': np.array([0.0, 0.0, 500.]),
            'o': np.array([0.0, 0.0, -500.]),
        }

        desired_force = forces.get(keypress, (0.0, 0.0, 0.0))
        self.body.addForce(desired_force)

        # Map keys to torques
        torques = {
            'f': np.array([-500., 0.0, 0.0]),
            'h': np.array([500., 0.0, 0.0]),
            't': np.array([0.0, 500., 0.0]),
            'g': np.array([0.0, -500., 0.0]),
            'r': np.array([0.0, 0.0, 500.]),
            'v': np.array([0.0, 0.0, -500.]),
        }

        desired_torque = torques.get(keypress, (0.0, 0.0, 0.0))
        self.body.addRelTorque(desired_torque)

    def set_up_ros(self):
        self.position_server = rospy.Service('sim/vehicle/set_pose', SimSetPose, self.set_pose_server)
        self.keypress_sub = rospy.Subscriber('sim/keypress', String, self.keypress_callback)

    def set_pose_server(self, srv):
        '''Set the pose of the submarine
        TODO: Make this an 'abstract' method of Entity, and assign each new Entity a name/id
        '''
        rospy.logwarn("Manually setting position of simulated vehicle")
        position = sub8_utils.rosmsg_to_numpy(srv.pose.position)
        self.body.setPosition(position)

        rotation_q = sub8_utils.rosmsg_to_numpy(srv.pose.orientation)
        rotation_norm = np.linalg.norm(rotation_q)

        velocity = sub8_utils.rosmsg_to_numpy(srv.twist.linear)
        angular = sub8_utils.rosmsg_to_numpy(srv.twist.angular)

        self.body.setLinearVel(velocity)
        self.body.setAngularVel(angular)

        # If the commanded rotation is valid
        if np.isclose(rotation_norm, 1., atol=1e-2):
            self.body.setQuaternion(sub8_utils.normalize(rotation_q))
        else:
            rospy.logwarn("Commanded quaternion was not a unit quaternion, NOT setting rotation")

        return SimSetPoseResponse()

    def publish_pose(self):
        '''TODO:
            Publish velocity in body frame
        '''
        linear_vel = self.body.getRelPointVel((0.0, 0.0, 0.0))
        # TODO: Not 100% on this transpose
        angular_vel = self.orientation.dot(self.angular_vel)
        # angular_vel = self.orientation.transpose().dot(self.angular_vel)

        quaternion = self.body.getQuaternion()

        translation = self.body.getPosition()

        header = sub8_utils.make_header(frame='/map')

        pose = geometry.Pose(
            position=geometry.Point(*translation),
            orientation=geometry.Quaternion(quaternion[1], quaternion[2], quaternion[3], quaternion[0]),
        )

        twist = geometry.Twist(
            linear=geometry.Vector3(*linear_vel),
            angular=geometry.Vector3(*angular_vel)
        )

        odom_msg = Odometry(
            header=header,
            child_frame_id='/base_link',
            pose=geometry.PoseWithCovariance(
                pose=pose
            ),
            twist=geometry.TwistWithCovariance(
                twist=twist
            )
        )

        self.truth_odom_pub.publish(odom_msg)

    def publish_imu(self, dt):
        """Publishes imu sensor information - orientation, angular velocity, and linear acceleration"""
        orientation_matrix = self.pose[:3, :3]
        sigma = 0.01
        noise = np.random.normal(0.0, sigma, 3)

        # Work on a better way to get this
        # We can't get this nicely from body.getForce()
        g = self.body.vectorFromWorld(self.g)
        # Get current velocity of IMU in body frame
        cur_vel = np.array(self.body.vectorFromWorld(self.body.getRelPointVel(self.imu_position)))
        linear_acc = orientation_matrix.dot(cur_vel - self.last_vel) / dt
        linear_acc += g + (noise * dt)

        # TODO: Fix frame
        angular_vel = orientation_matrix.dot(self.body.getAngularVel()) + (noise * dt)

        header = sub8_utils.make_header(frame='/body')

        linear = geometry.Vector3(*linear_acc)
        angular = geometry.Vector3(*angular_vel)

        covariance = [sigma ** 2, 0., 0.,
                      0., sigma ** 2, 0.,
                      0., 0., sigma ** 2]

        orientation_covariance = [-1, 0., 0.,
                                  0., 0., 0.,
                                  0., 0., 0.]

        imu_msg = Imu(
            header=header,
            angular_velocity=angular,
            angular_velocity_covariance=covariance,
            linear_acceleration=linear,
            linear_acceleration_covariance=covariance,
            orientation_covariance=orientation_covariance
        )

        self.imu_sensor_pub.publish(imu_msg)

    def publish_dvl(self):
        """Publishes dvl sensor data - twist message, and array of 4 dvl velocities based off of ray orientations"""
        correlation = -1

        header = sub8_utils.make_header(frame='/body')

        vel_dvl_body = self.body.vectorFromWorld(self.body.getRelPointVel(self.dvl_position))

        velocity_measurements = []

        for ray in self.dvl_ray_orientations:
            velocity_measurements.append(VelocityMeasurement(
                direction=geometry.Vector3(*ray),
                velocity=np.dot(ray, vel_dvl_body),
                correlation=correlation
            ))

        dvl_msg = VelocityMeasurements(
            header=header,
            velocity_measurements=velocity_measurements
        )

        self.dvl_sensor_pub.publish(dvl_msg)

    def thrust_cb(self, msg):
        '''Command the thrusters'''
        self.last_cmd_time = rospy.Time.now()
        self.thrust_dict = {}
        for thrust_cmd in msg.thruster_commands:
            commanded_thrust = thrust_cmd.thrust
            # Clamp thrust, and add it to dictionary
            self.thrust_dict[thrust_cmd.name] = np.clip(commanded_thrust, *self.thrust_range)

    def step(self, dt):
        '''Ignore dt
            TODO: Check if thruster is underwater
        '''
        # If timeout, reset thrust commands
        if (rospy.Time.now() - self.last_cmd_time) > self._cmd_timeout:
            self.thrust_dict.clear()

        if self.pos[2] < 0:
            # Apply thruster force only if underwater
            for i, (name, rel_dir, rel_pos) in enumerate(self.thruster_list):
                thruster_force = self.thrust_dict.get(name, 0.0)
                body_force = rel_dir * thruster_force
                self.body.addRelForceAtRelPos(body_force, rel_pos)

        self.apply_buoyancy_force()
        self.apply_damping_force()
        # self.apply_damping_torque()
        self.last_force = np.array(self.body.getForce()) + (self.g * self.mass)

        self.publish_pose()
        self.publish_dvl()
        self.publish_imu(dt)

        # Sets the position and direction of the DVL sensor rays to point below the current position of the sub
        dvl_position = self.body.getRelPointPos(self.dvl_position)
        for ray_geom, ray_orientation in self.dvl_rays:
            world_ray_orientation = self.body.vectorToWorld(ray_orientation)
            ray_geom.set(dvl_position, world_ray_orientation)
        self.last_vel = np.array(self.body.getRelPointVel(self.imu_position))

    def get_dvl_range(self):
        """Returns the range of each ray, otherwise 0 represents failure to contact the floor
            For each dvl sensor ray object, collisions with the floor are detected,
                and parameters from the contact.getContactGeomParams() method
                are used to calculate the range of the dvl sensor to the floor
        """
        ranges = np.array([0.0, 0.0, 0.0, 0.0])
        for n, ray in enumerate(self.dvl_rays):
            for contact in ode.collide(ray, self.space):
                pos, normal, depth, geom1, geom2 = contact.getContactGeomParams()

                assert geom1 is ray, geom1
                if (geom2 is self.geom):
                    continue
                ranges[n] = np.min(depth, ranges[n])  # The closest object!
        return ranges
