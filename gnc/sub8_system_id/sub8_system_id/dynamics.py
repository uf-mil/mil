#!/usr/bin/env python
import rospy
from urdf_parser_py.urdf import URDF
import numpy as np


class SubjuGatorDynamics(object):
    '''
    Models the dynamcis of SubjuGator, providing a shared interface for nodes using dynamics or
    inverse dynamics.
    '''
    def __init__(self, mass, rotational_inertia, volume, drag_coeffs, height,
                 water_density, air_density, G):
        '''
        @param mass: mass in kg
        @param rotational_inertia: 3x3 rotational inertia matrix
        @param volume: volume of sub in m^3
        @param drag_coeffs: 6x1 numpy array, N/(m/s) drag coeffients for X, Y, Z, R, P, Y
        @param height: Height of the sub, meters.
        @param water_density: density of water in kg/m^3
        @param air_density: density of air in kg/m^3
        @param G: float, Acceleration due to gravity in negative z of world frame, m/s^2
        '''
        self.mass = mass
        self.volume = volume
        self.drag_coeffs = drag_coeffs
        self.G = G
        self.height = height
        self.water_density = water_density
        self.air_density = air_density

        # Form 6 dimensional rotational matrix from mass and rotational inertia
        self.inertia = np.zeros((6, 6), dtype=float)
        self.inertia[0:3, 0:3] = self.mass * np.eye(3)
        self.inertia[3:6, 3:6] = rotational_inertia
        self.inertia_inv = np.linalg.inv(self.inertia)

    @classmethod
    def from_ros_params(cls):
        '''
        Initialize class from URDF and /robot_paramters/ ros param set by upload.launch
        '''
        # Parse URDF of Sub for inertial information, etc
        urdf_string = rospy.get_param('/robot_description')
        urdf = URDF.from_xml_string(urdf_string)
        # Get root link of model
        root = urdf.link_map[urdf.get_root()]
        mass = root.inertial.mass
        rotational_inertia = np.array(root.inertial.inertia.to_matrix())
        drag_linear = np.array(rospy.get_param('/robot_parameters/drag/linear_coeffs'))
        drag_angular = np.array(rospy.get_param('/robot_parameters/drag/angular_coeffs'))
        drag_coeffs = np.hstack((drag_linear, drag_angular)).T
        volume = rospy.get_param('/robot_parameters/volume')
        height = rospy.get_param('/robot_parameters/height')
        water_density = rospy.get_param('/robot_parameters/fluid_density')
        air_density = rospy.get_param('/robot_parameters/air_density')
        G = rospy.get_param('/robot_parameters/G')
        return cls(mass, rotational_inertia, volume, drag_coeffs, height,
                   water_density, air_density, G)

    def drag(self, twist):
        '''
        Calculate force/torque due to drag
        @param twist: 6x1 linear + angular velocity in body frame, m/s or rad/s
        @return 6x1 force + torque due to drag in body frame, N or N-M
        '''
        return -(self.drag_coeffs * twist)

    def gravity_and_buoyancy(self, z, world_to_body):
        '''
        Calculate force/torque due to gravity and buoyancy
        @param z: z coordinate of the position of the Sub's center of mass in the world frame
        @param world_to_body: 3x3 rotation matrix to rotate world frame to body frame
        '''
        # Calculate effect of gravity
        gravity = - self.G * self.mass

        # Estimate proportion of Sub's volume which is above the water (for buoyancy calculation)
        height_above_water = min(self.height, max(0., 0.5 * self.height + z))
        if height_above_water == 0.:
            proportion_above_water = 0.
        else:
            proportion_above_water = height_above_water / self.height

        #proportion_above_water = 0.

        # Calculate the portion of buoyancy force above the water
        # https://en.wikipedia.org/wiki/Archimedes%27_principle
        buoyancy_above_water = (self.air_density * self.G * self.volume) * proportion_above_water
        # Calculate the portion of the buoyancy force below the water
        buoyancy_below_water = (self.water_density * self.G * self.volume) * (1. - proportion_above_water)

        # Combine buoyancy and gravity into a single force vector
        grav_buoyancy = np.zeros(6)
        grav_buoyancy[2] = buoyancy_above_water + buoyancy_below_water + gravity
        # Rotate it into the body frame to be used in total force calculation
        grav_buoyancy[:3] = world_to_body.dot(grav_buoyancy[:3])
        return grav_buoyancy

    def newton_euler_extra_term(self, twist):
        '''
        Calculates the "extra term" in the newton-euler equations for 3D acceleration
        @param twist: 6x1 linear + angular velocity in body frame, m/s or rad/s
        '''
        extra_term = np.zeros(6)
        extra_term[3:6] = np.cross(twist[3:6], self.inertia[3:6, 3:6].dot(twist[3:6]))
        return extra_term

    def inverse_dynamics(self, z, twist, world_to_body, wrench=np.zeros(6)):
        '''
        Calculates the acceleration of the Sub given a wrench, taking into account
        drag, buoyancy, and gravity.
        @param z: z coordinate of the position of the Sub's center of mass in the world frame
        @param twist: 6x1 linear + angular velocity in body frame, m/s or rad/s
        @param world_to_body: 3x3 rotation matrix to rotate world frame to body frame
        @param wrench: 6x1 force + torque additional applied to body from motors, etc
        @return 6x1 linear + angular acceleration in body frame, m/s^2 or rad/s^2
        '''
        total_wrench = wrench + self.drag(twist) + self.gravity_and_buoyancy(z, world_to_body)
        return self.inverse_dynamics_from_total_wrench(twist, total_wrench)

    def inverse_dynamics_from_total_wrench(self, twist, total_wrench):
        '''
        Calculates the acceleration of the Sub given the TOTAL force/torque on the body.
        @param twist: 6x1 linear + angular velocity in body frame, m/s or rad/s
        @param wrench: 6x1 total force + torque applied to body
        @return 6x1 linear + angular acceleration in body frame, m/s^2 or rad/s^2
        '''
        # Perform Newton-Euler equation to determine acceleration
        # https://en.wikipedia.org/wiki/Newton%E2%80%93Euler_equations#Center_of_mass_frame
        return self.inertia_inv.dot(total_wrench - self.newton_euler_extra_term(twist))

    def dynamics(self, twist, accel):
        '''
        Calcualtes the net force + torque which produced a given acceleration
        @param twist: 6x1 linear + angular velocity in body frame, m/s or rad/s
        @param accel: 6x1 linear + angular acceleration in body frame, m/s^2 or rad/s^2
        @return 6x1 force + torque in body frame, N or N-M
        '''
        return self.inertia.dot(accel) + self.newton_euler_extra_term(twist)
