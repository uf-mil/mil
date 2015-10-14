#!/usr/bin/env python
from sub8_sim_tools import rendering, physics
from sub8_sim_tools.physics import Sub8, Box, Sphere, Mesh
from sub8_sim_tools.meshes import Transdec
from sub8_sim_tools.meshes import Sub8 as Sub8Visual
from vispy import app, gloo
import numpy as np
import rospy
import sys


class SimWorld(rendering.Canvas):
    def __init__(self):
        rospy.init_node('simulator')
        rospy.on_shutdown(self.end)
        self.rendering_world = rendering.World()
        self.physics_world = physics.World()

        self.entity_pairs = []

        # Physics world starts with such a plane
        plane = self.rendering_world.add_plane((0.0, 0.0, 1.0), 100.0, 100.0, color=(0, 0, 255, 170))

        self.rendering_world.add_mesh(Transdec, (0.0, 0.0, 0.0),
                                      orientation=None, color=(155, 155, 100), shininess=3.0)
        self.physics_world.add_entity(Mesh, (0.0, 0.0, 0.0), 10., Transdec, body=False)

        plane.set_debug()
        super(self.__class__, self).__init__()
        self.view = np.array([
            [1.,     0.,      0., 0.],
            [0.,    0.5,  -0.866, 0.],
            [0.,  0.866,     0.5, 0.],
            [0., -1.685, -36.931, 1.]
        ])
       #  self.view = np.array([[ -0.98426778,  -0.09466092,   0.1491853 ,   0.        ],
       # [  0.17061886,  -0.28998841,   0.94167506,   0.        ],
       # [ -0.04587782,   0.95231425,   0.30157719,   0.        ],
       # [-24.24570675,   2.97674952,  -3.7736146 ,   1.        ]])

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
        visual = self.rendering_world.add_mesh(Sub8Visual, position,
                                      orientation=None, color=(20, 20, 20, 1), shininess=20)
        physical = self.physics_world.add_entity(Sub8, position)
        self.rendering_world.add_entity(rendering.Indicator, physical, radius=0.2)

        self.rendering_world.add_entity(rendering.Indicator, physical, get_param=lambda o: np.array(o.last_force), 
                                        radius=0.1, color=(200, 10, 0), scaling_factor=0.01)
        self.entity_pairs.append((visual, physical))

    def on_draw(self, event):
        gloo.set_viewport(0, 0, *self.size)
        gloo.clear(color=True, depth=True)
        self.rendering_world.draw(self.view)

    def step_physics(self, event):
        self.physics_world.step()
        for rendered, physical in self.entity_pairs:
            rendered.set_pose(physical.pose)


if __name__ == '__main__':
    sim = SimWorld()
    # box = sim.add_box((-3.0, -3.0, 5.0), 1, 3, 3, 3, (255., 0.0, 0.0))
    sphere = sim.add_sphere((-3.0, -3.0, 15.0), 4.2, 1.0, (255., 0.0, 0.0))
    sub = sim.add_sub((0.0, 0.0, 0.0))
    sim.rendering_world.add_point_light((0.0, 0.0, 0.0), (0.8, 0.8, 0.8))

    app.run()