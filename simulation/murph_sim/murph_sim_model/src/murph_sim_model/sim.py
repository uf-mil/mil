#!/usr/bin/env python
'''

This source is written for use in the Machine Intelligence Lab in the MAE
Department at the University of Florida. 
It is writen for use on the UF RobotX robot
It is released under the BSD license and is intended for university use
This code is provided "as is" and relies on specific hardware, use at your own risk

Title: RobotX Simulator
Start Date: Date

Author: Forrest Voight
Author email: 

Co-author: Zach Goins
Co-author email: zach.a.goins@gmail.com

CODE DETAILS -------------------------------------------------------------------

'''

from __future__ import division
import math
import numpy
import random
import traceback
import rospy
import ode
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from twisted.internet import protocol, reactor, task
import roslib
roslib.load_manifest('murph_sim_model')

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, Quaternion, WrenchStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import Header
from murph_sim_rendering import sim_rendering_helpers as srh
from murph_sim_rendering import sim_math_helpers as smh
from murph_sim_model import lidar, Boat, Buoys
from murph_sim_rendering.sim_math_helpers import v, V
from roboteq_msgs.msg import *
from murph_sim_model import devices


rospy.init_node('murph_sim_model')

initial_position = rospy.get_param('~initial_position')
boat_mass =  rospy.get_param('~boat_mass')
boat_length =  rospy.get_param('~boat_length')
boat_width = rospy.get_param('~boat_width')
boat_height = rospy.get_param('~boat_height')

class Sim(object):
    # Base class for whatever you are writing
    def __init__(self, boat_model):

        self.boat_length =  rospy.get_param('~boat_length')
        self.boat_width = rospy.get_param('~boat_width')
        self.boat_height = rospy.get_param('~boat_height')
        self.port_servo_x_offset = rospy.get_param('~port_servo_x_offset')
        self.port_servo_y_offset = rospy.get_param('~port_servo_y_offset')
        self.starboard_servo_x_offset = rospy.get_param('~starboard_servo_x_offset')
        self.starboard_servo_y_offset = rospy.get_param('~starboard_servo_y_offset')
        self.friction_coefficient_forward = rospy.get_param('~friction_coefficient_forward')
        self.friction_coefficient_forward_reduction = rospy.get_param('~friction_coefficient_forward_reduction')
        self.friction_coefficient_lateral = rospy.get_param('~friction_coefficient_lateral')
        self.friction_coefficient_lateral_reduction = rospy.get_param('~friction_coefficient_lateral_reduction')
        self.friction_coefficient_rotational = rospy.get_param('~friction_coefficient_rotational')
        self.friction_coefficient_rotational_reduction = rospy.get_param('~friction_coefficient_rotational_reduction')

        self.boat_model = boat_model

        self.world_time = reactor.seconds()

        self.thrusts = {2: 0, 3: 0}
        self.positions = {2: 0, 3: 0}

        self.clip = lambda x, (low, high): min(max(x, low), high)

        self.killed = False
        self.locked = False

        #self.boat_lidar = lidar.Lidar()

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.abs_odom_pub = rospy.Publisher('absodom', Odometry, queue_size=1)
        self.base_tf_br = tf.TransformBroadcaster()
        self.enu_tf_br = tf.TransformBroadcaster()

        rospy.Subscriber("/left_motor/cmd" , Command, self.left_motor_cb)
        rospy.Subscriber("/right_motor/cmd" , Command, self.right_motor_cb)


    def get_water_vel(self, pos):
        return v(0, 0, 0)
        return (pos % v(0, 0, 1))*math.e**(-pos.mag()/3)

    def buoyancy_force(self, depth, r):    
        inf = 1e1000
        assert math.isinf(inf)
        sphere_antiderivative = lambda h: -h**3*math.pi/3 + h*math.pi*r**2
        sphere_true_antiderivative = lambda h: sphere_antiderivative(self.clip(h, (-r, r)))
        vol_submerged = sphere_true_antiderivative(depth) - sphere_true_antiderivative(-inf)
        return 1000 * 9.81 * vol_submerged

    def left_motor_cb(self, msg):
        self.thrusts[2] = msg.setpoint
        
    def right_motor_cb(self, msg):
        self.thrusts[3] = msg.setpoint

    def world_tick(self):
        
        #if random.randrange(10) == 0:
          #boat_lidar.get_lidar_range() # Use and publish whoa!

        water_vel = self.get_water_vel(V(body.getPosition()))
        
        body.addForceAtRelPos((0, 0, self.buoyancy_force(-body.getPosition()[2], 0.22728849402137372)), (0, 0, .1))
        # the following frictional forces are from darsen's models
        #frictional force opposite to the velocity of the boat
        body_velocity=body.vectorFromWorld(body.getLinearVel())
        # adds a resistance force for the water proportional to the velocity (where [0] is x, [1] is y, and [2] is z). units in N/(m/s)
        friction_force_forward=self.friction_coefficient_forward*body_velocity[0]-self.friction_coefficient_forward_reduction*(-1.0*pow(body_velocity[0],2) if body_velocity[0]<0.0 else pow(body_velocity[0],2))
        friction_force_lateral=self.friction_coefficient_lateral*body_velocity[1]-self.friction_coefficient_lateral_reduction*(-1.0*pow(body_velocity[1],2) if body_velocity[1]<0.0 else pow(body_velocity[1],2))
        body.addRelForce([-friction_force_forward,-friction_force_lateral,body_velocity[2]*-40])
        # adds a angular resistance force for the water proportional to the velocity
        # angular_velocity is a 3-tuple ([0]=roll,[1]=pitch,[2]=yaw)
        angular_velocity=body.vectorFromWorld(body.getAngularVel())
        friction_force_roll=self.friction_coefficient_rotational*angular_velocity[0]-self.friction_coefficient_rotational_reduction*(-1.0*pow(angular_velocity[0],2) if angular_velocity[0]<0.0 else pow(angular_velocity[0],2))
        friction_force_pitch=self.friction_coefficient_rotational*angular_velocity[1]-self.friction_coefficient_rotational_reduction*(-1.0*pow(angular_velocity[1],2) if angular_velocity[1]<0.0 else pow(angular_velocity[1],2))
        friction_force_yaw=self.friction_coefficient_rotational*angular_velocity[2]-self.friction_coefficient_rotational_reduction*(-1.0*pow(angular_velocity[2],2) if angular_velocity[2]<0.0 else pow(angular_velocity[2],2))
        body.addRelTorque([-friction_force_roll,-friction_force_pitch,-friction_force_yaw])
        print "forward: ", friction_force_forward, " lateral: ", friction_force_lateral, " rotational: ", -friction_force_yaw
        #print thrusters
        # map the thrusters position on the boat
        self.boat_model.vectors = []
        # id2=starboard, id3=port
        if not self.killed:
            for thruster_id in [2, 3]:
                # set the thrusters position on the boat
                relpos = v(self.starboard_servo_x_offset, self.starboard_servo_y_offset, 0) if thruster_id == 2 else v(self.port_servo_x_offset, self.port_servo_y_offset, 0)
                angle = 0
                reldir = v(math.cos(angle), math.sin(angle), 0)
                force = self.thrusts[thruster_id]
                body.addRelForceAtRelPos(reldir*force, relpos)
                self.boat_model.vectors.append((relpos, relpos - .02*reldir*force))
                print "force: ", force, " angle: ", angle
        
        keys = pygame.key.get_pressed()
        for keycode, force in [
            (pygame.K_k, v(-50, 0, 0)),
            (pygame.K_i, v(+50, 0, 0)),
            (pygame.K_j, v(0, +50, 0)),
            (pygame.K_l, v(0, -50, 0)),
            (pygame.K_o, v(0, 0, +50)),
            (pygame.K_m, v(0, 0, -50)),
        ]:
            if keys[keycode]:
                body.addRelForce(force*(10 if keys[pygame.K_RSHIFT] else 1)*(.1 if keys[pygame.K_RCTRL] else 1))
        for keycode, torque in [
            (pygame.K_COMMA, v(-20, 0, 0)),
            (pygame.K_u, v(+20, 0, 0)),
            (pygame.K_h, v(0, +20, 0)),
            (pygame.K_SEMICOLON, v(0, -20, 0)),
            (pygame.K_0, v(0, 0, +20)),
            (pygame.K_n, v(0, 0, -20)),
        ]:
            if keys[keycode]:
                body.addRelTorque(torque*(10 if keys[pygame.K_RSHIFT] else 1)*(.1 if keys[pygame.K_RCTRL] else 1))
        
        
        if keys[pygame.K_1]:
            self.killed = True
        if keys[pygame.K_2]:
            self.killed = False
        global locked
        if keys[pygame.K_3]:
            locked = True
        if keys[pygame.K_4]:
            locked = False
        
        contactgroup = ode.JointGroup()
        
        if self.locked:
            j = ode.FixedJoint(world, contactgroup)
            j.attach(body, None)
            j.setFixed()
        
        near_pairs = []
        space.collide(None, lambda _, geom1, geom2: near_pairs.append((geom1, geom2)))
        for geom1, geom2 in near_pairs:
            for contact in ode.collide(geom1, geom2):
                contact.setBounce(0.2)
                contact.setMu(5000)
                j = ode.ContactJoint(world, contactgroup, contact)
                j.attach(geom1.getBody(), geom2.getBody())
        
        dt = 1/30
        self.world_time += dt

        world.step(dt)
        
        contactgroup.empty()

        pos = body.getPosition()
        q = V(body.getQuaternion())

        # Publish tf /enu
        self.enu_tf_br.sendTransform(
            translation = (pos[0], pos[1], pos[2]),
            rotation = (q[1], q[2], q[3], q[0]),
            time = rospy.Time(self.world_time),
            child = '/base_link',
            parent = '/enu')
        
        # Publish odom
        msg = Odometry()
        msg.header.stamp = rospy.Time(self.world_time)
        msg.header.frame_id = '/enu'
        msg.child_frame_id = '/base_link'
        msg.pose.pose.position = Point(*pos)
        msg.pose.pose.orientation = Quaternion(q[1], q[2], q[3], q[0])
       
        msg.twist.twist.linear = Vector3(*q.conj().quat_rot(body.getLinearVel()))
        msg.twist.twist.angular = Vector3(*q.conj().quat_rot(body.getAngularVel()))
        self.odom_pub.publish(msg)
        
        # XXX
        msg = Odometry()
        msg.header.stamp = rospy.Time(self.world_time)
        msg.header.frame_id = '/ecef'
        msg.child_frame_id = '/base_link'
        # ecef_loc = gps.ecef_from_latlongheight(math.radians(36.802002), math.radians(-76.191019), 7)
        enu_loc = numpy.array([53.6686215007, -20.8502282916, -0.0733864689281])
        # msg.pose.pose.position = Point(*gps.ecef_from_enu(enu_v=body.getPosition() - enu_loc, ecef_pos=ecef_loc) + ecef_loc)
        self.abs_odom_pub.publish(msg)
        
        reactor.callLater(max(0, self.world_time + dt - reactor.seconds()), self.world_tick)

def set_forward_view():
    glTranslate(-1, 0, -1)
    srh.rotate_to_body(body, inv=True)

def get_water_vel(pos):
    return v(0, 0, 0)
    return (pos % v(0, 0, 1))*math.e**(-pos.mag()/3)

def setup(world, body, mass, space, g_world, body_geom, i, c1, boat_model):
    world.setGravity((0, 0, -9.81))
    # this is the mass of the boat "boat_mass", this also sets the inertia based on a box with length_x, length_y, length_z
    mass.setBoxTotal(boat_mass, boat_length, boat_width, boat_height)
    body.setMass(mass)
    body.setPosition(initial_position)
    # the last unit is the initial rotation of the body position in radians
    body.setQuaternion(smh.axisangle_to_quat(v(0, 0, 1), math.pi))
    #body.setQuaternion(smh.axisangle_to_quat(v(0, 0, 1), 0))
    body_geom.setBody(body)
    lake_mesh = srh.mesh_from_obj(roslib.packages.resource_file('murph_sim_model', 'models', 'lake.obj'))
    lake_geom = ode.GeomTriMesh(lake_mesh.ode_trimeshdata, space)

    # TODO: Add obstacles here

    g_world.objs.append(lake_mesh)
    
    g_world.objs.append(boat_model)
    g_world.objs.append(srh.VectorField(get_water_vel))

    geoms = []
    buoy_list = [v(54.5, -33.9, 0), v(53.5, -37.4, 0), v(38.0, -29.6, 0), v(35.7, -33.5, 0)]
    for pos in buoy_list:
        newbuoy_mesh = srh.mesh_from_obj(roslib.packages.resource_file('murph_sim_model', 'models', 'green_buoy.obj'))
        newbuoy_mesh = newbuoy_mesh.translate(pos)
        g_world.objs.append(newbuoy_mesh)
        geoms.append(ode.GeomTriMesh(newbuoy_mesh.ode_trimeshdata, space))

    i.init(g_world)

if __name__ == "__main__":

    world = ode.World()
    body = ode.Body(world)
    mass = ode.Mass()
    space = ode.HashSpace()
    g_world = srh.World()
    body_geom = ode.GeomBox(space, (boat_length, boat_width, boat_height))
    i = srh.Interface()
    c1 = srh.Camera(g_world, "forward_camera", set_forward_view, body, fovy=90)
    boat_model = Boat(body)
    sim = Sim(boat_model)

    setup(world, body, mass, space, g_world, body_geom, i, c1, boat_model)

    def update():
        try:
            i.step()
            c1.step()
        except:
            traceback.print_exc()
            reactor.stop()

    reactor.callWhenRunning(task.LoopingCall(update).start, 1/24)
    reactor.callWhenRunning(sim.world_tick)

    reactor.callWhenRunning(lambda: rospy.core.add_client_shutdown_hook(lambda reason=None: reactor.callFromThread(reactor.stop)))
    reactor.run()
