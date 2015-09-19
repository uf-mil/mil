from sub8_sim_tools import rendering, physics
from vispy import app, gloo
import numpy as np


class SimWorld(rendering.Canvas):
    def __init__(self):
        dt = 0.1
        self.rendering_world = rendering.World()
        self.physics_world = physics.World(dt)

        self.entity_pairs = []
        # Physics world starts with this
        plane = self.rendering_world.add_plane((0.0, 0.0, -1.0), 20.0, 20.0)
        plane.set_debug()
        super(self.__class__, self).__init__()
        self.view = np.array([
            [1., 0.,   0.,   0.],
            [0., 0.5,  -0.8660254 ,   0.],
            [0., 0.8660254,   0.5       ,   0.],
            [0., -1.68578305, -36.93058273,   1.]
        ])

    def add_sphere(self, *args):
        self.physics_world.add

    def add_box(self, position, density, width, height, depth, color):
        rendering_box = self.rendering_world.add_box(position, width, height, depth, color)
        physics_box = self.physics_world.add_box(position, density, width, height, depth)
        self.entity_pairs.append((rendering_box, physics_box))

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
    bx = sim.add_box((0.0, 0.0, 15.0), 10, 3, 3, 3, (255., 255.0, 0.0))
    sim.rendering_world.add_point_light((0.0, 0.0, 0.0), (0.1, 0.8, 0.5))

    app.run()