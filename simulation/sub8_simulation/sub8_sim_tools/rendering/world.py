from __future__ import division
from sub8_sim_tools import Shaders, ShaderManager
from sub8_ros_tools import make_rotation, compose_transformation
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

    def __init__(self, mesh, position=(0.0, 0.0, 0.0), orientation=None, color=(255., 0., 0., 255.0), faces=None,
                 shader_manager=None):
        '''Orientation must be a 4x4 rotation matrix'''
        self.position = np.array(position, dtype=np.float32)
        if orientation is None:
            self.orientation = np.eye(4)
        else:
            assert orientation.shape == (4, 4), "Orientation must be a 4x4 numpy array"
            self.orientation = orientation

        if len(color) == 3:
            color = color + (255,)
        self.color = np.array(color, dtype=np.float32) / 255.  # Normalize to [0, 1]

        self.program = gloo.Program(self._vertex_shader, self._fragment_shader)
        self.program.bind(mesh)
        self.faces = gloo.IndexBuffer(faces)

        self.model = np.eye(4, dtype=np.float32)
        self.model = self.model.dot(translate(self.position))
        self.view = np.eye(4, dtype=np.float32)

        self.projection = perspective(30.0, 800 / float(800), 2.0, 500.0)
        self.program['u_model'] = self.model
        self.program['u_view'] = self.view
        self.program['u_projection'] = self.projection
        self.program['u_color'] = self.color
        self.children = []

    def add_child(self, child):
        self.children.append(child)

    def translate(self, position):
        self.model = self.model.dot(translate(position))

    def rotate(self, angle, axis):
        '''Angle in radians'''
        self.model = self.model.dot(rotate(angle, axis))

    def set_pose(self, matrix):
        self.model = matrix
        for child in self.children:
            child.set_pose(matrix)
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

    @classmethod
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
    _vertex_shader = Shaders.base_shaders['phong']['vertex']
    _fragment_shader = Shaders.base_shaders['phong']['fragment']

    def __init__(self, position, radius, color, shader_manager=None, **kwargs):
        sphere_mesh = geometry.create_sphere(
            rows=int(radius * 30),
            cols=int(radius * 30),
            radius=radius,
            **kwargs
        )

        sphere_buffer, faces = self.make_buffer(sphere_mesh)
        super(self.__class__, self).__init__(sphere_buffer, faces=faces, position=position, color=color)
        shader_manager.register_lighting_shader(self)
        self.program['u_shininess'] = 16.0
        self.program['u_specular_color'] = self.color[:3]


class Box(Entity):
    _vertex_shader = Shaders.lighting['lambert']['vertex']
    _fragment_shader = Shaders.lighting['lambert']['fragment']

    def __init__(self, position, width, height, depth, color, shader_manager=None):
        box_mesh, box_faces, _ = geometry.create_box(
            width, height, depth,
            width_segments=int(width * 5),
            height_segments=int(height * 5),
            depth_segments=int(depth * 5)
        )

        box_buffer = np.zeros(
            len(box_mesh['position']),
            dtype=[
                ('a_position', np.float32, 3),
                ('a_normal', np.float32, 3)
            ]
        )

        box_buffer['a_position'] = box_mesh['position']
        box_buffer['a_normal'] = box_mesh['normal']
        # No textures for now
        # vertex_buffer['a_texcoord'] = box_buffer['texcoord']

        super(self.__class__, self).__init__(gloo.VertexBuffer(box_buffer), faces=box_faces, position=position,
                                             color=color)


class Plane(Entity):
    _vertex_shader = Shaders.base_shaders['phong']['vertex']
    _fragment_shader = Shaders.base_shaders['phong']['fragment']

    def __init__(self, position, width, height, color=(0, 255, 0), orientation=None, shader_manager=None):
        '''TODO:
            - Add set plane normal
        '''
        plane_mesh, plane_faces, _ = geometry.create_plane(
            width, height,
            width_segments=int(width * 5),
            height_segments=int(height * 5),
        )

        plane_buffer = np.zeros(
            len(plane_mesh['position']),
            dtype=[
                ('a_position', np.float32, 3),
                ('a_normal', np.float32, 3)
            ]
        )
        plane_buffer['a_position'] = plane_mesh['position']
        plane_buffer['a_normal'] = plane_mesh['normal']
        super(self.__class__, self).__init__(gloo.VertexBuffer(plane_buffer), faces=plane_faces, position=position,
                                             color=color)

        self.set_debug()
        shader_manager.register_lighting_shader(self)
        self.program['u_shininess'] = 16.0
        self.program['u_specular_color'] = self.color[:3]


class Indicator(Entity):
    _vertex_shader = Shaders.indicators['thrust_indicator']['vertex']
    _fragment_shader = Shaders.indicators['thrust_indicator']['fragment']

    def __init__(self, physics_entity, radius, offset=np.eye(4), get_param=lambda physent: physent.velocity,
                 get_offset=None, color=(0, 255, 0), scaling_factor=1.0, rigid=False, shader_manager=None):
        self.physics_entity = physics_entity
        self.get_param_func = get_param
        self.scaling_factor = scaling_factor
        self.offset = offset
        self.rigid = rigid

        arrow_mesh = geometry.create_arrow(
            rows=int(30),
            cols=int(30),
            radius=radius,
            length=1,
            cone_radius=1.2 * radius
        )

        arrow_buffer, faces = self.make_buffer(arrow_mesh)
        super(self.__class__, self).__init__(arrow_buffer, faces=faces, color=color)

    def draw(self):
        # pose = self.physics_entity.pose
        pos = self.physics_entity.pos
        directed_quantity = self.get_param_func(self.physics_entity)
        norm = np.linalg.norm(directed_quantity)

        if np.isclose(norm, 0.0, atol=1e-3):
            return

        R = make_rotation(np.array([0.0, 0.0, 1.0]), directed_quantity)
        if not self.rigid:
            model = compose_transformation(np.transpose(R), pos)
        else:
            model = compose_transformation(R, (0, 0, 0)).dot(self.physics_entity.pose)

        self.program['u_model'] = self.offset.dot(model)
        self.program['u_length_scale'] = norm * self.scaling_factor

        # Draw
        self.program.draw('triangles', self.faces)


class Mesh(Entity):
    _vertex_shader = Shaders.base_shaders['phong']['vertex']
    _fragment_shader = Shaders.base_shaders['phong']['fragment']

    def __init__(self, mesh, position, orientation=None, color=(0, 255, 0), shininess=16.0, shader_manager=None):
        '''TODO: Make loading this consistent with everything else'''
        # Texcoords is always none in the current version of Vispy
        mesh_vertices, mesh_faces, mesh_normals, texcoords = mesh

        mesh_buffer = np.zeros(
            len(mesh_vertices),
            dtype=[
                ('a_position', np.float32, 3),
                ('a_normal', np.float32, 3),
            ]
        )

        mesh_buffer['a_position'] = mesh_vertices
        mesh_buffer['a_normal'] = mesh_normals
        super(self.__class__, self).__init__(
            gloo.VertexBuffer(mesh_buffer),
            faces=mesh_faces,
            position=position,
            orientation=orientation,
            color=color
        )
        self.program['u_shininess'] = shininess
        self.program['u_specular_color'] = self.color[:3]
        shader_manager.register_lighting_shader(self)


class World(object):

    def __init__(self):
        '''Future methods
        - Add light
        - Add mesh
        - Add ros-camera (How to handle rigid-body offsets?)
        '''
        self.entities = []
        self.my_lights = []
        self.shader_manager = ShaderManager()

    def add_sphere(self, position, radius, color, **kwargs):
        '''Add a sphere entity to the scene'''
        kwargs["shader_manager"] = self.shader_manager
        sphere = Sphere(position, radius, color, **kwargs)
        self.entities.append(sphere)
        return sphere

    def add_box(self, position, width, height, depth, color):
        '''Add a box entity to the scene'''
        box = Box(position, width, height, depth, color, shader_manager=self.shader_manager)
        self.entities.append(box)
        return box

    def add_plane(self, position, width, height, color=(0, 0, 255), orientation=None):
        plane = Plane(position, width, height, color, orientation, shader_manager=self.shader_manager)
        self.entities.append(plane)
        return plane

    def add_mesh(self, mesh, *args, **kwargs):
        kwargs["shader_manager"] = self.shader_manager
        mesh = Mesh(mesh, *args, **kwargs)
        self.entities.append(mesh)
        return mesh

    def add_entity(self, Entity_Type, *args, **kwargs):
        kwargs["shader_manager"] = self.shader_manager
        entity = Entity_Type(*args, **kwargs)
        self.entities.append(entity)
        return entity

    def add_camera(self, position, orientation, topic, projection=None):
        '''Add a ros-camera to view the scene
            TODO: Only draw when the frame is going to be needed
        '''
        raise(NotImplementedError('add_camera not implemented!'))
        # camera = Camera((640, 640), position, orientation, topic, projection=projection)
        # self.views.append(camera)
        # return camera

    def add_sonar(self, position, orientation, projection=None, maxdepth=100):
        '''Add a ros-sonar to view the scene'''
        raise(NotImplementedError('add_sonar not implemented!'))

    def add_point_light(self, position, intensity):
        '''Add a point-light to illuminate the scene
            Shading implemented:
                - Diffuse (Lambert)
                - Blinn-Phong
                - Multiple lights
            TODO:
                - Shadows
            '''
        self.shader_manager.get_lighting_manager().add_item(position, intensity)

    def set_view(self, view):
        for entity in self.entities:
            entity.set_view(view)

    def draw(self, cur_view):
        '''Todo:
            Swappable shaders'''
        # Draw the objects to the screen
        for entity in self.entities[::-1]:
            entity.set_view(cur_view)
            entity.draw()

        # for view in self.views:
        #     # Attach to view framebuffer
        #     with view.view_buffer:
        #         gloo.set_viewport(0, 0, *view.resolution)
        #         for entity in self.entities:
        #             entity.view(view.view)
        #             entity.draw()
