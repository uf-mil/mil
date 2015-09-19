#!/usr/bin/env python
'''Physics components for the Sub8 simulator, Simul8
Original Author: Annie Luc
'''
from __future__ import division
import random
import traceback
import numpy as np
import time

import sys
import os
import random
import time

import ode
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, Quaternion, WrenchStamped
from sub8_msgs.msg import Thrust, ThrusterCmd


class World(object):
    def __init__(self, dt):
        '''This section incorporates PyODE to define methods to create and draw three objects: a sphere, box, or cylinder.
        Some code borrowed from http://sourceforge.net/p/pyode/mailman/message/19674697/.
        '''
        self.dt = dt
        self.entities = []
        # Create a world object
        self.ode_world = ode.World()
        # self.ode_world.setGravity(np.array([0, 0, -9.81]))    
        self.ode_world.setGravity(np.array([0, 0, -1.81]))    

        self.ode_world.setERP(0.8)
        self.ode_world.setCFM(1E-5)
    
        # Create a space object
        self.space = ode.Space()
    
        # Create a plane geom which prevent the objects from falling forever
        self.floor = ode.GeomPlane(self.space, (0., 0., 1.), 0.)
        # ode.GeomPlane(self.space, (0., 1., 0.), 0.)
        self.contact_group = ode.JointGroup()

    def add_box(self, position, density, lx, ly, lz):
        box = Box(self.ode_world, self.space, position, density, lx, ly, lz)
        self.entities.append(box)
        return box

    def step(self, dt=None):
        if dt is None:
            dt = self.dt

        for i in range(2):
            self.space.collide((self.ode_world, self.contact_group), self.near_callback)
            self.ode_world.step(dt / 2)
            for entity in self.entities:
                entity.step(dt)
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
    def __init__(self, world, space):
        self.body = ode.Body(world)

    def step(self):
        raise(NotImplementedError("You must implement a step function!"))

    # def submerged_volume(self):
    #     '''TODO: Implement required submerged volume determination
    #        For now, spherical approximation is sufficient
    #     '''
    #     raise(NotImplementedError("You must implement a submerged volume getter!"))

    @property
    def pose(self):
        '''Construct a 4x4 homogeneous pose matrix,
        Pose changes frequently enough that we recompute each time
        '''
        pose = np.zeros((4, 4))
        orientation = np.array(self.body.getRotation(), dtype=np.float32)
        orientation = orientation.reshape(3, 3)
        position = self.body.getPosition()
        pose[:3, :3] = orientation
        pose[3, :3] = position
        pose[3, 3] = 1.
        return pose


class Box(Entity):
    def __init__(self, world, space, position, density, lx, ly, lz):
        self.body = ode.Body(world)
        self.body.setPosition(position)

        M = ode.Mass()
        M.setBox(density, lx, ly, lz)
        self.body.setMass(M)

        self.body.shape = "box"
        self.body.boxsize = (lx, ly, lz)

        self.geom = ode.GeomBox(space, lengths=(lx, ly, lz))
        self.geom.setBody(self.body)

    def step(self, dt):
        # TODO: Improve from spherical approximation
        # self.body.addForce(real_buoyancy_force(-self.position[2], self.body.boxsize[0] / 2.)) 

        # Damping force
        velocity = np.array(self.body.getLinearVel())
        norm_velocity = -100. * np.linalg.norm(velocity)
        self.body.addForce(norm_velocity * velocity)


class Sphere(Entity):
    def __init__(self, pos, density, radius):
        self.body = ode.Body(world)
        self.body.setPosition(pos)
        M = ode.Mass()
        M.setSphere(density, radius)
        self.body.setMass(M)

        # Set parameters for drawing the body
        self.body.shape = "sphere"
        self.body.radius = radius

        # Create a Sphere geom for collision detection
        self.geom = ode.GeomSphere(space, radius)
        self.geom.setBody(self.body)

    def step(self, dt):
        self.body.addForce(real_buoyancy_force(-self.pos[2], self.body.radius))

        # Damping
        damping_coeff = -100.
        velocity = np.array(self.self.body.getLinearVel())
        norm_velocity = damping_coeff * np.linalg.norm(velocity)
        self.body.addForce(norm_velocity * velocity)
        thruster_force(self.body, self);

    @property
    def pos(self):
        return self.body.getPosition()


class Mesh(Entity):
    def __init__(self, pos, density, radius, height, color):
        '''Class to represent a Mesh object. 
           For now the object follows the physics of a sphere object in PyODE.
        '''
        self.body = ode.Body(world)
        self.body.setPosition(pos)
        M = ode.Mass()
        M.setSphere(density, radius)
        self.body.setMass(M)

        # Set parameters for drawing the body
        self.body.shape = "mesh"
        self.body.radius = radius
        self.thrusters = 8 * [0]

        # Create a Sphere geom for collision detection
        self.geom = ode.GeomSphere(space, radius)
        self.geom.setBody(self.body)

    def step(self, dt):
        self.body.addForce(real_buoyancy_force(-self.pos[2], self.body.radius))
        self.body.addForce(-1000*np.linalg.norm(self.body.getLinearVel())*np.array(self.body.getLinearVel()))
        thruster_force(self.body, self);

    @property
    def pos(self):
        return self.body.getPosition()


class Thrusters(object):
    def __init__(self):
        self.names = ["FLV", "FLL", "FRV", "FRL", "BLV", "BLL", "BRV", "BRL"]
        self.forces = [0]*8           

    def updateValues(self, body):
        for i in forces:
            body.thrusters[i] = forces[i]

'''
Bouyancy force is defined as the weight of the water displaced. 
This can be represented as [volume of ball under water]*[density of the water]*[acceleration due to gravity], where the density of water is 1000.
Buoyancy force = (-pi/3*h^3 + pi*h^2*r)*g*density, where h is depth of the submerged water.
The buoyancy force used in each step is derived from the real buoyancy force, but is scaled by the max_force value
'''

def real_buoyancy_force(depth, r):
    sphere_antiderivative = lambda h: (-h**3) * (np.pi / 3) + (h * np.pi) * (r**2)
    sphere_true_antiderivative = lambda h: sphere_antiderivative(np.clip(h, (-r, r)))
    vol_submerged = sphere_true_antiderivative(depth) - sphere_true_antiderivative(-np.inf)
    return -1000 * np.array([0, 0, -9.81])  * vol_submerged

def buoyancy_force(depth, r, max_force):                                                    #Scales buoyancy force by the max_force value
    return real_buoyancy_force(depth, r) / 1000 * 9.8 * ((4 / 3) * np.pi * r**3) * max_force

def thruster_force(body, shape):
    #thrusters = [50,1,1,1,0,0,0,0]   #Represent the first 4 frontal thrusters and the 4 rear thrusters 
    #Need to check if there are values set for thrusters or not
    thruster_list = [
        ("FLV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, 0.16900, 0.0142])),
        ("FLL", np.array([-0.866,  0.5,  0]), np.array([ 0.2678, 0.27950, 0.0000])),
        ("FRV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, -0.1690, 0.0142])),
        ("FRL", np.array([-0.866, -0.5,  0]), np.array([ 0.2678, -0.2795, 0.0000])),
        ("BLV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, 0.16900, 0.0142])),
        ("BLL", np.array([ 0.866,  0.5,  0]), np.array([-0.2678, 0.27950, 0.0000])),
        ("BRV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, -0.1690, 0.0142])),
        ("BRL", np.array([ 0.866, -0.5,  0]), np.array([-0.2678, -0.2795, 0.0000])),
    ]
    for i, (name, reldir, relpos, fwdforce, revforce) in enumerate(thruster_list):
        body.addRelForceAtRelPos(reldir*shape.thrusters[i]*(fwdforce if shape.thrusters[i] > 0 else revforce), relpos)

'''Thruster message callback to update the thruster values of any object in the world.'''
#Need to check with Jacob to see if the thrusters will be changed, or appended based on the message
def thruster_callback(msg):
    defaultThrusters = Thrusters()
    index = 0

    for thruster in msg.thruster_commands:
        for name in defaultThrusters.names:
            if name == thruster.name:
                defaultThrusters.forces[index] = thruster.thrust
                index += 1
    testSphere.updateThrusters(defaultThrusters.forces)


# Some variables used inside the simulation loop
fps = 50
dt = 1.0 / fps
running = True

# idle callback
def _idlefunc ():
    glutPostRedisplay ()
    # Simulate
    n = 2
    for i in range(n):
        # Detect collisions and create contact joints
        space.collide((world,contactgroup), near_callback)

        # Simulation step
        testSphere.step(dt / n)
        testBox.step(dt / n)

        world.step(dt / n)

        # Remove all contact joints
        contactgroup.empty()

if __name__ == '__main__':
    testSphere = Sphere((-10*random.gauss(0,0.1),-2.0,-2*random.gauss(0,0.1)), 100, 1, (0,0,0))
    testBox = Box((random.gauss(0,0.1),1.0,random.gauss(0,0.1)), 50, 1, 1, 1, (.6,.2,1))

    ######################################################################
    '''This sections incorporates ROS to be able to send messages to control the thruster values
    as well as publish messages on body odometry'''
    
    rospy.init_node('subsim')                                   #Initialization of ROS node called 'subsim' for process
    
    #odom_pub = rospy.Publisher('/sim_odom', Odometry)          #Initializes a publisher to report odometry of the simulation submarine
    thruster_subscriber = rospy.Subscriber('/thrust', Thrust, thruster_callback)
    
    glutIdleFunc (_idlefunc)
    glutMainLoop ()

    rospy.spin()