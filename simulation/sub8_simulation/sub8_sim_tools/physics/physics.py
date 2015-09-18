#!/usr/bin/env python

from __future__ import division                             #Allows proper division

import math                                                 #Access to mathematical functionss
import random                                               #Acess to pseudo-random number generators for various distributions
import traceback                                            #Interface to extract, format, and print stack traces of Python programs
import numpy as np
import time

import sys, os, random, time #For PyODE 3 tutorial

import ode                                                  #Library to simulate rigid body dynamics
import pygame                                               #Python modules designed to write video games
from pygame.locals import *
from OpenGL.GL import *                                     #API for rendering 2D and 3D vector graphics
from OpenGL.GLU import *                                    #Computer graphics library for OpenGL
from OpenGL.GLUT import *

import rospy                                                #Python client library for ROS
from nav_msgs.msg import Odometry                           #Messages to store an estimate of the position and velocity of a robot in free space
from geometry_msgs.msg import Point, Vector3, Quaternion, WrenchStamped    #Messages for common geometric primitives such as points, vectors, and poses
from sub8_msgs.msg import Thrust, ThrusterCmd

from subsim import devices                                  #SubjuGator/subsim/src/subsim/devices.py


"""
Total force = sum of each thruster = F times direction vector to get force vector
Torque is equal to moment (position cross force dot position
"""

# prepare_GL
def prepare_GL():
    """Prepare drawing.
    """

    # Viewport
    glViewport(0,0,1080,840)

    # Initialize
    glClearColor(0.2,0.7,0.9,0)             #Makes background watery blue
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST)
    glDisable(GL_LIGHTING)
    glEnable(GL_LIGHTING)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_FLAT)

    # Projection
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    #Sets up perspective projection matrix (field of view angle in the y direction, 
    #aspect ratio (width to height), distance from viewer to near clipping plane, 
    #distance from viewer to far clipping plane)
    gluPerspective (45,1,.2,20)       

    # Initialize ModelView matrix
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    # Light source
    glLightfv(GL_LIGHT0,GL_POSITION,[0,0,1,0])      #Makes initial light source directional, parallel to, and in the direction of the - z axis
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])       #Diffuses RGBA intensity of the light
    glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])      #Specifies the specular RGBA intensity of the light
    glEnable(GL_LIGHT0)

    # View transformation
    gluLookAt (3.5, 7.5, 7.5, 0.5, 0.5, 0, 0, 0, 1) #Creates a viewing matrix derived from an eye point, a reference point indicating the center of the scene, and an UP vector

######################################################################
"""This section incorporates PyODE to define methods to create and draw three objects: a sphere, box, or cylinder.
Some code borrowed from http://sourceforge.net/p/pyode/mailman/message/19674697/.
"""

# Create a world object
world = ode.World()
world.setGravity(np.array([0, 0, -9.81]))    
world.setERP(0.8)           #Set the global ERP value, that controls how much error correction is performed in each time step.
world.setCFM(1E-5)          #Set the global CFM (constraint force mixing) value.

# Create a space object
space = ode.Space()

# Create a plane geom which prevent the objects from falling forever
floor = ode.GeomPlane(space, (0,0,1), 0)

class Box(object):
    def __init__(self, pos, density, lx, ly, lz, color):
        self.body = ode.Body(world)
        self.body.setPosition(pos)
        M = ode.Mass()
        M.setBox(density, lx, ly, lz)
        self.body.setMass(M)

        # Set parameters for drawing the body
        self.body.shape = "box"
        self.body.boxsize = (lx, ly, lz)
        self.thrusters = 8*[0]

        # Create a box geom for collision detection
        self.geom = ode.GeomBox(space, lengths=self.body.boxsize)
        self.geom.setBody(self.body)

    def step(self, dt):
        self.body.addForce(real_buoyancy_force(-self.pos[2], self.body.boxsize[0] / 2)) #Need to approximate radius by length of box
        self.body.addForce(-1000*np.linalg.norm(self.body.getLinearVel())*np.array(self.body.getLinearVel()))
        thruster_force(self.body, self);

    def updateThrusters(self, values):
        index = 0
        for i in values:
            self.thrusters[index] = values[index]
            index += 1

    @property
    def pos(self):
        return self.body.getPosition()
    
    def draw(self):
        x,y,z = self.body.getPosition()
        R = self.body.getRotation()
        rot = [R[0], R[3], R[6], 0.,
           R[1], R[4], R[7], 0.,
           R[2], R[5], R[8], 0.,
           x, y, z, 1.0]
        glPushMatrix()
        glMultMatrixd(rot)
        sx,sy,sz = self.body.boxsize
        glScale(sx, sy, sz)
        glutSolidCube(1)
        glPopMatrix()


class Sphere(object):
    def __init__(self, pos, density, radius, color):
        self.body = ode.Body(world)
        self.body.setPosition(pos)
        M = ode.Mass()
        M.setSphere(density, radius)
        self.body.setMass(M)

        # Set parameters for drawing the body
        self.body.shape = "sphere"
        self.body.radius = radius
        self.thrusters = 8*[0]

        # Create a Sphere geom for collision detection
        self.geom = ode.GeomSphere(space, radius)
        self.geom.setBody(self.body)

    def step(self, dt):
        self.body.addForce(real_buoyancy_force(-self.pos[2], self.body.radius))
        self.body.addForce(-1000*np.linalg.norm(self.body.getLinearVel())*np.array(self.body.getLinearVel()))
        thruster_force(self.body, self);

    def updateThrusters(self, values):
        index = 0
        for i in values:
            self.thrusters[index] = values[index]
            index += 1

    @property
    def pos(self):
        return self.body.getPosition()
    
    def draw(self):
        x,y,z = self.body.getPosition()
        R = self.body.getRotation()
        rot = [R[0], R[3], R[6], 0.,
           R[1], R[4], R[7], 0.,
           R[2], R[5], R[8], 0.,
           x, y, z, 1.0]
        glPushMatrix()
        glMultMatrixd(rot)
        radius = self.body.radius
        glutSolidSphere(radius,20,20)
        glPopMatrix()

"""Class to represent a Mesh object. 
For now the object follows the physics of a sphere object in PyODE."""
class Mesh(object):
    def __init__(self, pos, density, radius, height, color):

        #Assume 

        #Will need to define position, vertexes
        #Will need to define point of center of mass
        #Will need to decide ODE body to represent Mesh (density, radius, etc.)
        self.body = ode.Body(world)
        self.body.setPosition(pos)
        M = ode.Mass()
        M.setSphere(density, radius)
        self.body.setMass(M)

        # Set parameters for drawing the body
        self.body.shape = "mesh"
        self.body.radius = radius
        self.thrusters = 8*[0]

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
    
    def draw(self):
        x,y,z = self.body.getPosition()
        R = self.body.getRotation()
        rot = [R[0], R[3], R[6], 0.,
           R[1], R[4], R[7], 0.,
           R[2], R[5], R[8], 0.,
           x, y, z, 1.0]
        glPushMatrix()
        glMultMatrixd(rot)
        radius = self.body.radius
        glutSolidSphere(radius,20,20)
        glPopMatrix()


class Thrusters(object):
    def __init__(self):
        self.names = ["FLV", "FLL", "FRV", "FRL", "BLV", "BLL", "BRV", "BRL"]
        self.forces = [0]*8           

    def updateValues(self, body):
        for i in forces:
            body.thrusters[i] = forces[i]

"""Use this code to set random orientation and position: 
body.setPosition( (random.gauss(0,0.1),3.0,random.gauss(0,0.1)) )
theta = random.uniform(0,2*pi)
ct = cos (theta)
st = sin (theta)
body.setRotation([ct, 0., -st, 0., 1., 0., st, 0., ct])
"""

######################################################################
"""This sections defines the physics incorporated into each world step on each object. 
The collision callback function is defined here.
Thruster net force is defined here.
"""

# Collision callback
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method.
    This function checks if the given geoms do collide and
    creates contact joints if they do.
    """
    # Check if the objects do collide
    contacts = ode.collide(geom1, geom2)
    # Create contact joints
    world,contactgroup = args
    for c in contacts:
        c.setBounce(.4)
        c.setMu(5000)
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

"""
Bouyancy force is defined as the weight of the water displaced. 
This can be represented as [volume of ball under water]*[density of the water]*[acceleration due to gravity], where the density of water is 1000.
Buoyancy force = (-pi/3*h^3 + pi*h^2*r)*g*density, where h is depth of the submerged water.
The buoyancy force used in each step is derived from the real buoyancy force, but is scaled by the max_force value
"""

clip = lambda x, (low, high): min(max(x, low), high)        #Clip represents an anonymous function that returns the middle value between high, low, and x

def real_buoyancy_force(depth, r):                          #Calculates buoyancy force of submarine based on depth in water
    inf = 1e1000
    assert math.isinf(inf)
    sphere_antiderivative = lambda h: (-h**3)*(math.pi/3) + (h*math.pi)*(r**2)              #Represents the volume of the sphere in water
    sphere_true_antiderivative = lambda h: sphere_antiderivative(clip(h, (-r, r)))          #Calculates sphere_antiderivative with middle value of h, -r, and r
    vol_submerged = sphere_true_antiderivative(depth) - sphere_true_antiderivative(-inf)    #Volume submerged is determined by the difference in volume of sphere in water versus the volume of total submarine
    return -1000 * np.array([0, 0, -9.81])  * vol_submerged                                          #Buoyancy force is returned based on formula in above comments, with -1000 as density of water           

def buoyancy_force(depth, r, max_force):                                                    #Scales buoyancy force by the max_force value
    return real_buoyancy_force(depth, r)/1000 * 9.8 * 4/3*math.pi*r**3 * max_force

def thruster_force(body, shape):
    #thrusters = [50,1,1,1,0,0,0,0]   #Represent the first 4 frontal thrusters and the 4 rear thrusters 
    #Need to check if there are values set for thrusters or not
    thruster_list = [
        ("FLV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, 0.16900, 0.0142]), 21.57, 17.98), # FLV
        ("FLL", np.array([-0.866,  0.5,  0]), np.array([ 0.2678, 0.27950, 0.0000]), 21.57, 17.98), # FLL
        ("FRV", np.array([ 0.000,  0.0, -1]), np.array([ 0.1583, -0.1690, 0.0142]), 21.57, 17.98), # FRV
        ("FRL", np.array([-0.866, -0.5,  0]), np.array([ 0.2678, -0.2795, 0.0000]), 21.57, 17.98), # FRL
        ("BLV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, 0.16900, 0.0142]), 21.57, 17.98), # BLV
        ("BLL", np.array([ 0.866,  0.5,  0]), np.array([-0.2678, 0.27950, 0.0000]), 21.57, 17.98), # BLL
        ("BRV", np.array([ 0.000,  0.0,  1]), np.array([-0.1583, -0.1690, 0.0142]), 21.57, 17.98), # BRV
        ("BRL", np.array([ 0.866, -0.5,  0]), np.array([-0.2678, -0.2795, 0.0000]), 21.57, 17.98), # BRL
    ]
    for i, (name, reldir, relpos, fwdforce, revforce) in enumerate(thruster_list):
        body.addRelForceAtRelPos(reldir*shape.thrusters[i]*(fwdforce if shape.thrusters[i] > 0 else revforce), relpos)

"""Thruster message callback to update the thruster values of any object in the world."""
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

######################################################################

# Initialize Glut
glutInit ([])

# Open a window
glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE)

glutInitWindowPosition (0, 0);
glutInitWindowSize (1080, 840);
glutCreateWindow ("testode")

# A joint group for the contact joints that are generated whenever
# two bodies collide
contactgroup = ode.JointGroup()

# Some variables used inside the simulation loop
fps = 50
dt = 1.0/fps
running = True

# keyboard callback
def _keyfunc (c, x, y):
    sys.exit (0)

glutKeyboardFunc (_keyfunc)

# draw callback
def _drawfunc ():
    # Draw the scene
    prepare_GL()
    
    testSphere.draw()
    testBox.draw()

    glutSwapBuffers ()

glutDisplayFunc (_drawfunc)

# idle callback
def _idlefunc ():
    glutPostRedisplay ()
    # Simulate
    n = 2
    for i in range(n):
        # Detect collisions and create contact joints
        space.collide((world,contactgroup), near_callback)

        # Simulation step
        testSphere.step(dt/n)
        testBox.step(dt/n)

        world.step(dt/n)

        # Remove all contact joints
        contactgroup.empty()

if __name__ == '__main__':
    testSphere = Sphere((-10*random.gauss(0,0.1),-2.0,-2*random.gauss(0,0.1)), 100, 1, (0,0,0))
    testBox = Box((random.gauss(0,0.1),1.0,random.gauss(0,0.1)), 50, 1, 1, 1, (.6,.2,1))

    ######################################################################
    """This sections incorporates ROS to be able to send messages to control the thruster values
    as well as publish messages on body odometry"""
    
    rospy.init_node('subsim')                                   #Initialization of ROS node called 'subsim' for process
    
    #odom_pub = rospy.Publisher('/sim_odom', Odometry)          #Initializes a publisher to report odometry of the simulation submarine
    thruster_subscriber = rospy.Subscriber('/thrust', Thrust, thruster_callback)
    
    glutIdleFunc (_idlefunc)
    glutMainLoop ()

    rospy.spin()
    ######################################################################
