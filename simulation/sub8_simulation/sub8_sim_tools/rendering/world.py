from __future__ import division
from sub8_sim_tools import Shaders
from vispy import geometry, gloo
import numpy as np
from vispy.util.transforms import perspective, translate, rotate


class Entity(object):
    _debug = False
    _vertex_shader = Shaders.passthrough['mesh']['vertex']
    _fragment_shader = Shaders.passthrough['mesh']['fragment']

    @classmethod
    def set_debug(self, debug=True):
        '''Enable debug mode:
            - Draw outlines of faces in red'''
        self.debug = debug

    def __init__(self, mesh, position=(0.0, 0.0, 0.0), orientation=None, color=(255., 0., 0.), faces=None):
        '''Orientation must be a 4x4 rotation matrix'''
        self.position = np.array(position, dtype=np.float32)
        if orientation is None:
            self.orientation = np.eye(4)
        else:
            assert orientation.shape == (4, 4), "Orientation must be a 4x4 numpy array"
            self.orientation = orientation

        self.color = np.array(color, dtype=np.float32) / 255.  # Normalize to [0, 1]

        self.program = gloo.Program(self._vertex_shader, self._fragment_shader)
        # self.program.bind(gloo.VertexBuffer(mesh))
        self.program.bind(mesh)
        self.faces = gloo.IndexBuffer(faces)

        self.model = np.eye(4, dtype=np.float32)
        self.model = self.model.dot(translate(self.position))
        self.view = np.eye(4, dtype=np.float32)
        self.projection = perspective(30.0, 800 / float(800), 2.0, 100.0)
        self.program['u_model'] = self.model
        self.program['u_view'] = self.view
        self.program['u_projection'] = self.projection
        self.program['u_color'] = self.color

    def translate(self, position):
        self.model = self.model.dot(translate(position))

    def rotate(self, angle, axis):
        '''Angle in radians'''
        rad_angle = np.radians(angle)
        self.model = self.model.dot(rotate(angle, axis))

    def set_pose(self, matrix):
        self.model = matrix
        self.program['u_model'] = self.model

    def set_view(self, matrix):
        self.view = matrix
        self.program['u_view'] = self.view

    def draw(self):
        # Compute Normals
        normal = np.transpose(np.linalg.inv(np.dot(self.view, self.model)))
        self.program['u_normal'] = normal

        # Draw
        self.program.draw('triangles', self.faces)

        # Draw debug frame
        if self._debug:
            self.program['u_color'] = (0.5, 0.25, 0.25)
            self.program.draw('lines', self.faces)
            self.program['u_color'] = self.color

    def make_buffer(self, mesh):
        assert isinstance(mesh, geometry.meshdata.MeshData)
        faces = mesh.get_faces()
        vertices = mesh.get_vertices()
        normals = mesh.get_vertex_normals()

        vertex_buffer = np.zeros(
            len(vertices), 
            dtype=[
                ('a_position', np.float32, 3),
                ('a_normal', np.float32, 3)
            ]
        )

        vertex_buffer['a_position'] = vertices
        vertex_buffer['a_normal'] = normals

        return gloo.VertexBuffer(vertex_buffer), faces


class Sphere(Entity):
    _vertex_shader = Shaders.lighting['lambert']['vertex']
    _fragment_shader = Shaders.lighting['lambert']['fragment']

    def __init__(self, position, radius, color, **kwargs):
        sphere_mesh = geometry.create_sphere(
            rows=int(radius * 30),
            cols=int(radius * 30),
            radius=radius,
            **kwargs
        )
        sphere_buffer, faces = self.make_buffer(sphere_mesh)
        super(self.__class__, self).__init__(sphere_buffer, faces=faces, position=position, color=color)


class Box(Entity):
    _vertex_shader = Shaders.lighting['lambert']['vertex']
    _fragment_shader = Shaders.lighting['lambert']['fragment']

    def __init__(self, position, width, height, depth, color):
        cube_mesh, cube_faces, _ = geometry.create_box(
            width, height, depth,
            width_segments=int(width * 5),
            height_segments=int(height * 5),
            depth_segments=int(depth * 5)
        )

        cube_buffer = np.zeros(
            len(cube_mesh['position']), 
            dtype=[
                ('a_position', np.float32, 3),
                ('a_normal', np.float32, 3)
            ]
        )

        cube_buffer['a_position'] = cube_mesh['position']
        cube_buffer['a_normal'] = cube_mesh['normal'] 
        # No textures for now
        # vertex_buffer['a_texcoord'] = cube_buffer['texcoord'] 

        super(self.__class__, self).__init__(gloo.VertexBuffer(cube_buffer), faces=cube_faces, position=position, color=color)


class World(object):
    def __init__(self):
        '''Future methods
        - Add light
        - Add mesh
        - Add ros-camera (How to handle rigid-body offsets?)
        '''
        # Set up vispy objects
        # Initialize list of objects to render
        # Initialize global shader
        # World.add_object
        self.entities = []

    def add_sphere(self, position, radius, color, **kwargs):
        '''Add a sphere entity to the scene'''
        sphere = Sphere(position, radius, color, **kwargs)
        self.entities.append(sphere)
        return sphere

    def add_box(self, position, width, height, depth, color):
        '''Add a box entity to the scene'''
        box = Box(position, width, height, depth, color)
        self.entities.append(box)
        return box

    def add_camera(self, position, orientation, topic, projection=None):
        '''Add a ros-camera to view the scene'''
        raise(NotImplementedError('add_camera not implemented!'))
        camera = Camera((640, 640), position, orientation, topic, projection=projection)
        self.views.append(camera)

    def add_sonar(self, position, orientation, projection=None, maxdepth=100):
        '''Add a ros-sonar to view the scene'''
        raise(NotImplementedError('add_sonar not implemented!'))

    def add_point_light(self, position, intensity):
        '''Add a point-light to illuminate the scene
            Shading implemented: 
                - Diffuse (Lambert)
            TODO:
                - Blinn-Phong
            '''
        for entity in self.entities:
            entity.program['u_light_position'] = position
            entity.program['u_light_intensity'] = intensity

    def set_view(self, view):
        for entity in self.entites:
            entity.set_view(view)

    def draw(self, cur_view):
        '''Todo:
            Swappable shaders'''
        # Draw the objects to the screen
        for entity in self.entities:
                entity.set_view(cur_view)
                entity.draw()


        # for view in self.views:
        #     # Attach to view framebuffer
        #     with view.view_buffer:
        #         gloo.set_viewport(0, 0, *view.resolution)
        #         for entity in self.entities:
        #             entity.view(view.view)
        #             entity.draw()
