#!/usr/bin/env python
'''Physics components for the Sub8 simulator, Simul8
Original Author: Annie Luc
'''
from __future__ import division
import numpy as np
import ode
import rospy


class Constants(object):
    density_water = 999.97  # kg / m**3
    g = -9.81  # m / s**2, gravitational constant
    # g = -0.01


class World(object):
    def __init__(self, dt=(1 / 45.)):
        '''This section incorporates PyODE to define methods to create and draw three objects: a sphere, box, or cylinder.
            Some code borrowed from http://sourceforge.net/p/pyode/mailman/message/19674697/.
        '''
        self.dt = dt
        self.entities = []
        # Create a world object
        self.ode_world = ode.World()
        self.ode_world.setGravity(np.array([0, 0, Constants.g]))

        try:
            self.ode_world.setAngularDamping(0.2)
        except AttributeError:
            rospy.logerr("SIM ERROR: You need to re-run the install script, or manually fix pyode")
            rospy.logerr("SIM ERROR: Killing simulation")
            exit(0)

        # self.ode_world.setLinearDamping(0.2)
        self.ode_world.setERP(0.8)
        self.ode_world.setCFM(1E-5)

        # Create a space object
        self.space = ode.Space()

        self.contact_group = ode.JointGroup()
        self.ode_world.step(self.dt)

    def add_entity(self, Entity_Type, *args, **kwargs):
        '''Add an arbitrary entity
        Example:
            Say we want to add a Sphere to my physics world
            from physics import Sphere, World
            w = World(dt=0.1)
            w.add_entity(Sphere, postion, density, radius)

        NOTE that we did not instantiate a Sphere, there was no Sphere(~~~)
        We just passed the Sphere class, not a Sphere instance
        This is for greater extensibility without repeated code
        '''
        entity = Entity_Type(self.ode_world, self.space, *args, **kwargs)
        self.entities.append(entity)
        self.ode_world.step(self.dt)
        return entity

    def step(self, dt=None):
        if dt is None:
            dt = self.dt

        for entity in self.entities:
            entity.step(dt)

        self.space.collide((self.ode_world, self.contact_group), self.near_callback)
        self.ode_world.step(dt)

        # Remove all contact joints
        self.contact_group.empty()

    def near_callback(self, (world, contact_group), geom1, geom2):
        '''Callback function for the collide() method.
        This function checks if the given geoms do collide and
        creates contact joints if they do.
        '''
        # Check if the objects do collide
        contacts = ode.collide(geom1, geom2)
        # Create contact joints
        for c in contacts:
            c.setBounce(.4)
            c.setMu(5000)
            j = ode.ContactJoint(world, contact_group, c)
            j.attach(geom1.getBody(), geom2.getBody())


class Entity(object):
    _linear_damping_coeff = -0.2
    _rotational_damping_coeff = -0.2

    def __init__(self, world, space):
        self.body = ode.Body(world)

    def step(self):
        raise(NotImplementedError("You must implement a step function!"))

    @property
    def pos(self):
        return np.array(self.body.getPosition(), dtype=np.float32)

    @property
    def orientation(self):
        r = np.array(self.body.getRotation(), dtype=np.float32)
        R = r.reshape(3, 3)
        return R

    @property
    def pose(self):
        '''Construct a 4x4 homogeneous pose matrix,
        Pose changes frequently enough that we recompute each time
        Pose is in the world frame
        '''
        pose = np.zeros((4, 4))
        position = self.body.getPosition()
        # Don't use self.pos, to avoid a wasteful array cast
        pose[:3, :3] = self.orientation
        pose[3, :3] = position
        pose[3, 3] = 1.
        return pose

    @property
    def velocity(self):
        '''Returns world-frame velocity, a tuple of BOTH (linear_vel, angular_vel)'''
        linear_vel = np.array(self.body.getLinearVel(), dtype=np.float32)
        return linear_vel

    @property
    def angular_vel(self):
        angular_vel = np.array(self.body.getAngularVel(), dtype=np.float32)
        return angular_vel

    @property
    def submerged_volume(self):
        '''Assume water is at z = 0
        Volume of sphere: (4 / 3)pi * r**3
        Volume of a spherical cap of height h: (1 / 3)pi * h**2 * (3r - h)
        '''
        h = np.clip(self.pos[2] + self.radius, 0.0, 2 * self.radius)
        sphere_volume = (4. / 3.) * (np.pi * (self.radius ** 3.))
        above_water_volume = (1 / 3) * np.pi * (h ** 2) * ((3 * self.radius) - h)
        submerged_volume = sphere_volume - above_water_volume
        return submerged_volume

    def apply_buoyancy_force(self):
        '''Apply buoyancy force based on submerged volume approximation
        '''
        # Negative, because g < 0
        buoyancy_force = -Constants.density_water * Constants.g * self.submerged_volume
        self.body.addForce((0.0, 0.0, buoyancy_force))

    def apply_damping_force(self):
        '''Apply a quadratic damping force'''
        velocity = np.array(self.body.getLinearVel(), dtype=np.float32)
        norm_velocity = np.linalg.norm(velocity)
        if not np.isclose(norm_velocity, 0.0):
            force = velocity * norm_velocity * self._linear_damping_coeff
            self.body.addForce(force)

    def apply_damping_torque(self):
        '''Apply a linear rotational damping torque'''
        angular_velocity = np.array(self.body.getAngularVel(), dtype=np.float32)
        norm_velocity = self._rotational_damping_coeff * np.linalg.norm(angular_velocity)
        self.body.addTorque(norm_velocity * angular_velocity)


class Box(Entity):
    _linear_damping_coeff = -20  # TODO: Estimate area
    _rotational_damping_coeff = -0.5  # TODO: Estimate area

    def __init__(self, world, space, position, density, lx, ly, lz):
        self.body = ode.Body(world)
        self.body.setPosition(position)

        M = ode.Mass()
        M.setBox(density, lx, ly, lz)
        self.body.setMass(M)

        self.geom = ode.GeomBox(space, lengths=(lx, ly, lz))
        self.geom.setBody(self.body)

    def step(self, dt):
        # TODO: Improve from spherical approximation
        self.apply_damping_force()
        self.apply_damping_torque()


class Sphere(Entity):
    _linear_damping_coeff = -20.
    _rotational_damping_coeff = -0.05

    def __init__(self, world, space, position, density, radius):
        self.body = ode.Body(world)
        self.body.setPosition(position)
        M = ode.Mass()
        M.setSphere(density, radius)
        self.body.setMass(M)
        self.radius = radius
        self._linear_damping_coeff *= radius
        self._rotational_damping_coeff *= radius

        # Create a Sphere geom for collision detection
        self.geom = ode.GeomSphere(space, radius)
        self.geom.setBody(self.body)

    def step(self, dt):
        self.apply_damping_force()
        self.apply_damping_torque()
        self.apply_buoyancy_force()


class Mesh(Entity):
    def __init__(self, world, space, position, density, mesh_info):
        '''Mesh object - Dynamics are *unsupported* right now
            This spawns a static object that can only be collided with and does not move
            TODO: Fix that
        '''
        # Create a mesh geometry for collision detection
        mesh_vertices, mesh_faces, mesh_normals, texcoords = mesh_info
        meshdata = ode.TriMeshData()
        meshdata.build(mesh_vertices, mesh_faces)
        self.geom = ode.GeomTriMesh(meshdata, space)
        self.body = ode.Body(world)

    def step(self, dt):
        # self.apply_damping_force()
        # self.apply_damping_torque()
        pass
