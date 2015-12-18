import numpy as np
from sub8_ros_tools import make_rotation, compose_transformation
from sub8_sim_tools import rendering
from sub8_sim_tools.physics import Sub8
from sub8_sim_tools.meshes import Sub8 as Sub8Visual


class Sub(object):
    def __init__(self, rendering_world, physics_world, position, thrust_indicators=True):
        self.rendering_world = rendering_world
        self.physics_world = physics_world
        self._physical = self.physics_world.add_entity(Sub8, position)
        self._visual = self.make_visual(self._physical, position, thrust_indicators)
        self.thruster_list = [
            ("FLV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, 0.16900, 0.0142])),  # noqa
            ("FLL", np.array([-0.866,  0.5,  0]), np.array([ 0.2678, 0.27950, 0.0000])),  # noqa
            ("FRV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, -0.1690, 0.0142])),  # noqa
            ("FRL", np.array([-0.866, -0.5,  0]), np.array([ 0.2678, -0.2795, 0.0000])),  # noqa
            ("BLV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, 0.16900, 0.0142])),  # noqa
            ("BLL", np.array([ 0.866,  0.5,  0]), np.array([-0.2678, 0.27950, 0.0000])),  # noqa
            ("BRV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, -0.1690, 0.0142])),  # noqa
            ("BRL", np.array([ 0.866, -0.5,  0]), np.array([-0.2678, -0.2795, 0.0000])),  # noqa
        ]

    @property
    def physical(self):
        return self._physical

    @property
    def visual(self):
        return self._visual

    def make_visual(self, physical, position, thrust_indicators=False):
        visual = self.rendering_world.add_mesh(Sub8Visual, position,
                                               orientation=None, color=(20, 20, 20, 1), shininess=20)

        self.rendering_world.add_entity(rendering.Indicator, physical, radius=0.05)

        self.rendering_world.add_entity(rendering.Indicator, physical, get_param=lambda o: np.array(o.last_force),
                                        radius=0.025, color=(200, 10, 0), scaling_factor=0.01)

        class ThrustGetter(object):
            '''This is a pretty garbage thing, and will be replaced by the scenegraph system
            '''
            def __init__(self, thruster_name, rdir):
                self.thruster_name = thruster_name
                self.rdir = rdir

            def __call__(self, entity):
                return entity.thrust_dict.get(self.thruster_name, 0.0) * np.array([0.0, 0.0, 1.0])

        if thrust_indicators:
            # add the thrust indicators
            for name, rel_direction, rel_position in self.thruster_list:

                R = make_rotation(np.array([0.0, 0.0, 1.0]), rel_direction)
                transformation = compose_transformation(np.transpose(R), rel_position)

                t = ThrustGetter(name, rel_direction)
                self.rendering_world.add_entity(rendering.Indicator, physical,
                                                get_param=t, rigid=True,
                                                offset=transformation, radius=0.01, scaling_factor=0.01,
                                                color=(0, 40, 200)
                                                )

        return visual
