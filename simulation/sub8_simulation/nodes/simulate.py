#!/usr/bin/env python
from sub8_sim_tools import rendering, physics
from sub8_sim_tools.physics import Sub8, Box, Sphere, Mesh
from sub8_sim_tools.meshes import Transdec
from sub8_sim_tools.meshes import Sub8 as Sub8Visual
from sub8_ros_tools import make_rotation, compose_transformation
from rosgraph_msgs.msg import Clock
from vispy import app, gloo
import numpy as np
import rospy
import sys
import time


class SimWorld(rendering.Canvas):
    def __init__(self):
        rospy.init_node('simulator')
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        self.time_acceleration = rospy.get_param('time_acceleration', 1.0)
        self.physics_dt = rospy.get_param('physics_dt', 1 / 30.)
        self.draw = rospy.get_param('draw', True)

        rospy.on_shutdown(self.end)
        self.rendering_world = rendering.World()
        self.physics_world = physics.World(self.physics_dt)

        self.entity_pairs = []

        # Physics world starts with such a plane
        self.rendering_world.add_plane((0.0, 0.0, 1.0), 100.0, 100.0, color=(0, 0, 255, 170))

        self.rendering_world.add_mesh(Transdec, (0.0, 0.0, 0.0),
                                      orientation=None, color=(155, 155, 100), shininess=3.0)
        self.physics_world.add_entity(Mesh, (0.0, 0.0, 0.0), 10., Transdec)

        self.start_time = None
        super(self.__class__, self).__init__(self.time_acceleration, show_window=self.draw, physics_dt=self.physics_dt)

        self.view = np.array([
            [1.,     0.,      0., 0.],
            [0.,    0.5,  -0.866, 0.],
            [0.,  0.866,     0.5, 0.],
            [0., -1.685, -36.931, 1.]
        ])

    def end(self):
        '''This sometimes generates errors due to unfavorable ROS-Vispy interactions
            when SIGINT is issued'''
        self.close()
        sys.exit()

    def add_sphere(self, position, density, radius, color):
        visual = self.rendering_world.add_sphere(position, radius, color)
        physical = self.physics_world.add_entity(Sphere, position, density, radius)
        self.entity_pairs.append((visual, physical))

    def add_box(self, position, density, width, height, depth, color):
        visual = self.rendering_world.add_box(position, width, height, depth, color)
        physical = self.physics_world.add_entity(Box, position, density, width, height, depth)
        self.entity_pairs.append((visual, physical))

    def add_sub(self, position):
        '''
        TODO:
            This should be some kind of seperate widget, but alas, it is currently just jammed in here
        '''
        visual = self.rendering_world.add_mesh(Sub8Visual, position,
                                      orientation=None, color=(20, 20, 20, 1), shininess=20)
        physical = self.physics_world.add_entity(Sub8, position)
        self.rendering_world.add_entity(rendering.Indicator, physical, radius=0.05)

        self.rendering_world.add_entity(rendering.Indicator, physical, get_param=lambda o: np.array(o.last_force),
                                        radius=0.025, color=(200, 10, 0), scaling_factor=0.01)

        thruster_list = [
            ("FLV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, 0.16900, 0.0142])),  # flake8: noqa
            ("FLL", np.array([-0.866,  0.5,  0]), np.array([ 0.2678, 0.27950, 0.0000])),  # flake8: noqa
            ("FRV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, -0.1690, 0.0142])),  # flake8: noqa
            ("FRL", np.array([-0.866, -0.5,  0]), np.array([ 0.2678, -0.2795, 0.0000])),  # flake8: noqa
            ("BLV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, 0.16900, 0.0142])),  # flake8: noqa
            ("BLL", np.array([ 0.866,  0.5,  0]), np.array([-0.2678, 0.27950, 0.0000])),  # flake8: noqa
            ("BRV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, -0.1690, 0.0142])),  # flake8: noqa
            ("BRL", np.array([ 0.866, -0.5,  0]), np.array([-0.2678, -0.2795, 0.0000])),  # flake8: noqa
        ]

        # add the thrust indicators
        for name, rel_direction, rel_position in thruster_list:

            R = make_rotation(np.array([0.0, 0.0, 1.0]), rel_direction)
            transformation = compose_transformation(np.transpose(R), rel_position)

            class ThrustGetter(object):
                def __init__(self, thruster_name, rdir):
                    self.thruster_name = thruster_name
                    self.rdir = rdir

                def __call__(self, entity):
                    return entity.thrust_dict.get(self.thruster_name, 0.0) * np.array([0.0, 0.0, 1.0])

            t = ThrustGetter(name, rel_direction)
            self.rendering_world.add_entity(rendering.Indicator, physical,
               get_param=t, rigid=True,
               offset=transformation, radius=0.01, scaling_factor=0.01, color=(0, 40, 200)
            )
        self.entity_pairs.append((visual, physical))

    def on_draw(self, event):
        '''TODO:
            Make a better way to skip drawing...this is a hack at best
        '''
        gloo.set_viewport(0, 0, *self.size)
        gloo.clear(color=True, depth=True)
        self.rendering_world.draw(self.view)

    def step_physics(self, event):
        if self.start_time is None:
            self.start_time = time.time()
        self.clock += self.physics_dt
        self.clock_pub.publish(Clock(clock=rospy.Time(self.clock)))

        self.physics_world.step()
        if self.draw:
            for rendered, physical in self.entity_pairs:
                rendered.set_pose(physical.pose)


if __name__ == '__main__':
    sim = SimWorld()
    sphere = sim.add_sphere((-3.0, -3.0, 15.0), 4.2, 1.0, (255., 0.0, 0.0))
    sub = sim.add_sub((0.0, 0.0, 0.0))
    sim.rendering_world.add_point_light((0.0, 0.0, 0.0), (0.8, 0.8, 0.8))

    app.run()