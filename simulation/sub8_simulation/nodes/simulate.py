#!/usr/bin/env python
from sub8_sim_tools import rendering, physics
from sub8_sim_tools.widgets import Sub
from sub8_sim_tools.physics import Box, Sphere, Mesh
from sub8_sim_tools.meshes import Transdec
from rosgraph_msgs.msg import Clock
from vispy import app, gloo
import numpy as np
import rospy


class SimWorld(rendering.Canvas):

    def __init__(self):
        rospy.init_node('simulator')
        self.clock_pub = rospy.Publisher('/clock', Clock, queue_size=1)
        self.time_acceleration = rospy.get_param('time_acceleration', 1.0)
        self.physics_dt = rospy.get_param('physics_dt', 1 / 100.)
        self.draw = rospy.get_param('draw', True)

        self.rendering_world = rendering.World()
        self.physics_world = physics.World(self.physics_dt)

        self.entity_pairs = []

        # Physics world starts with such a plane
        self.rendering_world.add_plane((0.0, 0.0, 1.0), 100.0, 100.0, color=(0, 0, 255, 170))

        self.rendering_world.add_mesh(Transdec, (0.0, 0.0, 0.0), orientation=None, color=(155, 155, 100), shininess=3.0)
        self.physics_world.add_entity(Mesh, (0.0, 0.0, 0.0), 10., Transdec)

        self.rendering_world.add_point_light((0.0, 1.0, 0.0), (0.0, 0.0, 0.0))

        super(self.__class__, self).__init__(
            self.time_acceleration, show_window=self.draw, physics_dt=self.physics_dt)

        self.view = np.array([
            [1.,     0.,      0., 0.],  # noqa
            [0.,    0.5,  -0.866, 0.],  # noqa
            [0.,  0.866,     0.5, 0.],  # noqa
            [0., -1.685, -36.931, 1.],  # noqa
        ])

    def add_sphere(self, position, density, radius, color):
        visual = self.rendering_world.add_sphere(position, radius, color)
        physical = self.physics_world.add_entity(
            Sphere, position, density, radius)
        self.entity_pairs.append((visual, physical))

    def add_box(self, position, density, width, height, depth, color):
        visual = self.rendering_world.add_box(
            position, width, height, depth, color)
        physical = self.physics_world.add_entity(
            Box, position, density, width, height, depth)
        self.entity_pairs.append((visual, physical))

    def add_sub(self, position):
        self.sub = Sub(self.rendering_world, self.physics_world, position, thrust_indicators=False)
        visual = self.sub.visual  # property
        physical = self.sub.physical  # property
        self.entity_pairs.append((visual, physical))

    def on_draw(self, event):
        '''TODO:
            Make a better way to skip drawing...this is a hack at best
        '''
        gloo.set_viewport(0, 0, *self.size)
        gloo.clear(color=True, depth=True)
        self.rendering_world.draw(self.view)

    def step_physics(self, event):
        self.clock += self.physics_dt
        self.clock_pub.publish(Clock(clock=rospy.Time(self.clock)))
        self.clock += self.physics_dt

        self.physics_world.step()
        if self.draw:
            for rendered, physical in self.entity_pairs:
                rendered.set_pose(physical.pose)


if __name__ == '__main__':
    sim = SimWorld()
    sphere = sim.add_sphere((-3.0, -3.0, 15.0), 4.2, 1.0, (255., 0.0, 0.0))
    sim.rendering_world.add_point_light((0.0, 0.0, 1.0), (0.2, 0.2, 0.2))
    sub = sim.add_sub((0.0, 0.0, -5.0))
    sim.rendering_world.add_point_light((0.0, 1.0, 1.0), (0.2, 0.2, 0.2))
    app.run()
