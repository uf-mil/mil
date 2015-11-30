from __future__ import division

import itertools
import math
import os
import sys
import time
import random

import numpy
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.arrays import vbo
import pygame

import roslib
roslib.load_manifest('murph_sim_rendering')
import rospy
import tf
from tf import transformations
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3, Quaternion

from sim_math_helpers import v, V
import sim_math_helpers

@apply
class GLMatrix(object):
    def __enter__(self):
        glPushMatrix()
    
    def __exit__(self, type, value, traceback):
        glPopMatrix()

def angleaxis_matrix(angle, (x, y, z)):
    s = math.sin(angle)
    c = math.cos(angle)
    return numpy.array([
        [(1-c)*x*x + c,   (1-c)*y*x - z*s, (1-c)*z*x + y*s, 0],
        [(1-c)*x*y + z*s, (1-c)*y*y + c,   (1-c)*z*y - x*s, 0],
        [(1-c)*x*z - y*s, (1-c)*y*z + x*s, (1-c)*z*z + c,   0],
        [0,               0,                0,              1],
    ])

def euler_matrix(yaw, pitch, roll):
    return numpy.dot(angleaxis_matrix(roll, (1, 0, 0)), numpy.dot(angleaxis_matrix(pitch, (0, 1, 0)), angleaxis_matrix(yaw, (0, 0, 1))))

def rotate_vec(vec, m):
    x = numpy.dot((vec[0], vec[1], vec[2], 1), m)
    return V(x[:3])/x[3]

class Material(object):
    pass # no defaults
    def apply(self):
        assert self.illum == [2]
        
        assert len(self.Ns) == 1
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, self.Ks + [1])
        glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, self.Ns*3 + [1])
        
        assert len(self.Kd) == 3
        assert len(self.d) == 1
        glColor4f(*self.Kd + self.d)
        
        # ignoring Ka because of blender weirdness
        # ignoring Ni (index of refraction)
def parse_mtl(filename):
    current_mtl = None
    res = {}
    for line in open(filename):
        line = line.strip('\r\n')
        if '#' in line:
            line = line[:line.index("#")]
        if not line: continue
        line = line.split(' ')
        #print line
        if line[0] == "newmtl":
            current_mtl = line[1]
            res[current_mtl] = Material()
            res[current_mtl].name = current_mtl
        else:
            setattr(res[current_mtl], line[0], map(float, line[1:]))
    return res

def mesh_from_obj(filename):
    vertices = []
    texcoords = []
    normals = []
    indices = []
    materials = []
    ignore = False
    current_mtl = None
    for line in open(filename):
        line = line.strip('\r\n')
        if '#' in line:
            line = line[:line.index("#")]
        if not line: continue
        line = line.split(' ')
        if line[0] == "o":
            if line[1].startswith('background_') or line[1].startswith('marker_'):
                ignore = True
            else:
                ignore = False
        elif line[0] == "v":
            vertices.append(V(float(x) for x in line[1:]))
        elif line[0] == "vt":
            texcoords.append(V(float(x) for x in line[1:]))
        elif line[0] == "vn":
            normals.append(V(float(x) for x in line[1:]))
        elif line[0] == "mtllib":
            mtllib = parse_mtl(os.path.join(os.path.dirname(os.path.abspath(filename)), line[1]))
        elif line[0] == 'usemtl':
            current_mtl = line[1]
        elif line[0] == "f":
            if not ignore:
                indices.append(tuple(tuple(int(y)-1 if y else None for y in (x.split('/')+['',''])[:3]) for x in line[1:]))
                materials.append(mtllib[current_mtl])
        else:
            print line
    return Mesh(vertices, texcoords, normals, indices, materials)

class Mesh(object):
    def __init__(self, vertices, texcoords, normals, indices, materials):
        self.vertices = vertices
        self.texcoords = texcoords
        self.normals = normals
        self.indices = indices
        self.materials = materials
        
        self.draw2 = None
    
    def generate(self):
        vbos = {}
        for mtl, x in itertools.groupby(zip(self.indices, self.materials), lambda (triangle, mtl): mtl):
            #print mtl, x
            vecs = []
            for triangle, mtl in x:
                assert len(triangle) == 3
                for vert_index, tex_index, normal_index in triangle:
                    vecs.append([
                        self.vertices[vert_index],
                        self.normals[normal_index] if normal_index is not None else (0, 0, 0),
                        self.texcoords[tex_index] if tex_index is not None else (0, 0, 0),
                    ])
            _vbo = vbo.VBO(numpy.array(vecs, 'f'))
            vbo_count = len(vecs)
            vbos[mtl] = _vbo, vbo_count
        
        def draw():
            glDisable(GL_CULL_FACE)
            for mtl, (vbo, vbo_count) in vbos.iteritems():
                mtl.apply()
                vbo.bind()
                glEnableClientState(GL_VERTEX_ARRAY)
                glEnableClientState(GL_NORMAL_ARRAY)
                glEnableClientState(GL_TEXTURE_COORD_ARRAY)
                glVertexPointer(3, GL_FLOAT, 36, vbo)
                glNormalPointer(GL_FLOAT, 36, vbo + 12)
                glTexCoordPointer(3, GL_FLOAT, 36, vbo + 24)
                glDrawArrays(GL_TRIANGLES, 0, vbo_count)
                glDisableClientState(GL_VERTEX_ARRAY)
                glDisableClientState(GL_NORMAL_ARRAY)
                glDisableClientState(GL_TEXTURE_COORD_ARRAY)
                vbo.unbind()
            glEnable(GL_CULL_FACE)
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0, 0, 0, 1])
        return draw
    
    def draw(self):
        if self.draw2 is None:
            self.draw2 = self.generate()
        self.draw2()
    
    @property
    def ode_trimeshdata(self):
        import ode
        x = ode.TriMeshData()
        x.build(self.vertices, [[p[0] for p in t] for t in self.indices])
        return x
    
    def translate(self, dx):
        return Mesh([x+dx for x in self.vertices], self.texcoords, self.normals, self.indices, self.materials)
    
    def rotate(self, q):
        return Mesh([q.quat_rot(x) for x in self.vertices], self.texcoords, [q.quat_rot(x) for x in self.normals], self.indices, self.materials)
    
    def scale(self, s):
        return Mesh([x.scale(s) for x in self.vertices], self.texcoords, self.normals, self.indices, self.materials)


def rotate_to_body(body, inv=False):
    R = body.getRotation()
    p = body.getPosition()
    rot = [[R[0], R[3], R[6], 0.],
        [R[1], R[4], R[7], 0.],
        [R[2], R[5], R[8], 0.],
        [p[0], p[1], p[2], 1.]]
    if inv:
        rot = numpy.linalg.inv(rot)
    rot = list(rot[0])+list(rot[1])+list(rot[2])+list(rot[3])
    glMultMatrixd(rot)

class VectorField(object):
    def __init__(self, func):
        self.func = func
    
    def draw(self):
        glBegin(GL_LINES)
        for x in xrange(-10, 10+1):
            for y in xrange(-10, 10+1):
                pos = v(x, y, 0)
                vel = self.func(pos)
                glColor3d(1, 0, 0)
                glVertex3f(*pos-vel/2)
                glColor3d(1, 1, 0)
                glVertex3f(*pos+vel/2)
        glEnd()

def perspective(fovy, aspect, zNear):
    f = 1/math.tan(math.radians(fovy)/2)
    glMultMatrixf([
        [f/aspect, 0, 0, 0],
        [ 0, f, 0, 0],
        [0, 0, -1, -1],
        [0, 0, -2*zNear, 0]
    ])

class World(object):
    def __init__(self):
        self.objs = []
    
    def step(self, dt):
        for obj in self.objs:
            if hasattr(obj, "step"):
                obj.step(dt)
    
    def draw(self):
        t = time.time()
        pos = V(numpy.linalg.inv(glGetFloatv(GL_MODELVIEW_MATRIX))[3,0:3])
        
        glClearColor(0, 0, 1, 1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, [0, 0, 0, 1])
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0, 0, 0, 1])
        glLightfv(GL_LIGHT0, GL_POSITION, [math.sin(t/5)*100, math.cos(t/5)*100, 100, 1])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0, 0, 0, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [.5, .5, .5, 1])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [.5, .5, .5, 1])
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [.5, .5, .5])
        
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_CULL_FACE)
        
        for obj in self.objs:
            obj.draw()
        '''
        q = gluNewQuadric()
        for i in xrange(100):
            with GLMatrix:
                glColor3d(random.random(), random.random(), random.random())
                glTranslate(*pos+5*V(random.gauss(0, 1) for i in xrange(3)))
                gluSphere(q, .5, 10, 5)
        '''
        # sun
        with GLMatrix:
            glTranslate(math.sin(t/5)*100, math.cos(t/5)*100, 100)
            q = gluNewQuadric()
            glDisable(GL_LIGHTING)
            glColor3f(1, 1, 1)
            gluSphere(q, 10, 20, 20)
            glEnable(GL_LIGHTING)
        
        # water
        glDisable(GL_LIGHTING)
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        if pos[2] > 0:
            glBegin(GL_TRIANGLE_FAN)
            glNormal3f(0, 0, 1)
            glColor4f(0, 0, 0.5, 0.5)
            glVertex4f(pos[0], pos[1], min(0, pos[2] - 0.2), 1)
            glVertex4f(-1, -1, 0, 0)
            glVertex4f(+1, -1, 0, 0)
            glVertex4f(+1, +1, 0, 0)
            glVertex4f(-1, +1, 0, 0)
            glVertex4f(-1, -1, 0, 0)
            glEnd()
        else:
            glBegin(GL_TRIANGLE_FAN)
            glNormal3f(0, 0, -1)
            glColor4f(0, 0, 0.5, 0.5)
            glVertex4f(pos[0], pos[1], max(0, pos[2] + 0.2), 1)
            glVertex4f(-1, -1, 0, 0)
            glVertex4f(-1, +1, 0, 0)
            glVertex4f(+1, +1, 0, 0)
            glVertex4f(+1, -1, 0, 0)
            glVertex4f(-1, -1, 0, 0)
            glEnd()
        
        glDisable(GL_BLEND)
        
        glEnable(GL_LIGHTING)
        
        # underwater color
        if pos[2] < 0:
            glPushMatrix()
            glLoadIdentity()
            
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glMatrixMode(GL_MODELVIEW)
            
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glBegin(GL_QUADS)
            glColor4f(0, 0, 1, 0.1)
            glVertex3f(-1, -1, 0)
            glVertex3f(+1, -1, 0)
            glVertex3f(+1, +1, 0)
            glVertex3f(-1, +1, 0)
            glEnd()
            glDisable(GL_BLEND)
            
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_LIGHTING)
            
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            glMatrixMode(GL_MODELVIEW)
            
            glPopMatrix()

tf_br = tf.TransformBroadcaster()

class Camera(object):
    def __init__(self, world, name, set_pose_func, base_link_body, fovy=90, aspect=640/480):
        self.world = world
        self.name = name
        self.set_pose_func = set_pose_func
        self.base_link_body = base_link_body
        self.fovy = fovy
        self.aspect = aspect
        
        self.image_pub = rospy.Publisher('/%s/image_rect_color' % name, Image, queue_size=1)
        self.info_pub = rospy.Publisher('/%s/camera_info' % name, CameraInfo, queue_size=1)
    
    def step(self):
        t = rospy.Time.now()
        _, _, oldwidth, oldheight = glGetFloatv(GL_VIEWPORT)
        height = min(oldheight, int(oldwidth / self.aspect + .5))
        width = int(self.aspect * height + .5)
        glViewport(0, 0, width, height)
        
        msg = CameraInfo()
        msg.header.stamp = t
        msg.header.frame_id = '/' + self.name
        msg.height = height
        msg.width = width
        f = 1/math.tan(math.radians(self.fovy)/2)*height/2
        msg.P = [
            f, 0, width/2-.5, 0,
            0, f, height/2-.5, 0,
            0, 0, 1, 0,
        ]
        self.info_pub.publish(msg)
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        perspective(self.fovy, width/height, 0.1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # rotates into the FLU coordinate system
        glMultMatrixf([
            [ 0., 0.,-1., 0.],
            [-1., 0., 0., 0.],
            [ 0., 1., 0., 0.],
            [ 0., 0., 0., 1.]
        ])
        # after that, +x is forward, +y is left, and +z is up
        self.set_pose_func()
        
        with GLMatrix:
            rotate_to_body(self.base_link_body)
            glcamera_from_body = glGetFloatv(GL_MODELVIEW_MATRIX).T
        camera_from_body = numpy.array([ # camera from glcamera
            [1, 0, 0, 0],
            [0,-1, 0, 0],
            [0, 0,-1, 0],
            [0, 0, 0, 1],
        ]).dot(glcamera_from_body)
        body_from_camera = numpy.linalg.inv(camera_from_body)
        tf_br.sendTransform(transformations.translation_from_matrix(body_from_camera),
                     transformations.quaternion_from_matrix(body_from_camera),
                     t,
                     "/" + self.name,
                     "/base_link")
        
        if not self.image_pub.get_num_connections():
            glViewport(0, 0, oldwidth, oldheight)
            return
        
        self.world.draw()
        
        x = glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, outputType=None)
        x = numpy.reshape(x, (height, width, 4))
        x = x[::-1]
        
        msg = Image()
        msg.header.stamp = t
        msg.header.frame_id = '/' + self.name
        msg.height = height
        msg.width = width
        msg.encoding = 'rgba8'
        msg.is_bigendian = 0
        msg.step = width * 4
        msg.data = x.tostring()
        self.image_pub.publish(msg)
        
        glViewport(0, 0, oldwidth, oldheight)

class Interface(object):
    def init(self, world, pos=v(0, -10, 2)):
        self.world = world
        self.pos = pos
        
        self.display_flags = pygame.DOUBLEBUF|pygame.OPENGL|pygame.RESIZABLE
        self.display = pygame.display.set_mode((640, 480), self.display_flags)
        self.clock = pygame.time.Clock()
        
        self.pitch = self.yaw = 0
        self.grabbed = False
        
        self.fovy = 100
    
    def step(self):
        dt = self.clock.tick()/1000
        
        for event in pygame.event.get():
            if event.type == pygame.MOUSEMOTION:
                if self.grabbed:
                    self.yaw += event.rel[0]/100
                    
                    self.pitch += -event.rel[1]/100
                    # caps it to a quarter turn up or down
                    self.pitch = min(max(self.pitch, -math.pi/2), math.pi/2)
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_TAB:
                    self.grabbed = not self.grabbed
                    pygame.event.set_grab(self.grabbed)
                    pygame.mouse.set_visible(not self.grabbed)
                
                elif event.key == pygame.K_q:
                    sys.exit()
            
            elif event.type == pygame.QUIT:
                sys.exit()
            
            elif event.type == pygame.VIDEORESIZE:
                self.display = pygame.display.set_mode(event.size, self.display_flags)
                glViewport(0, 0, self.display.get_width(), self.display.get_height())
        
        rot_matrix = euler_matrix(self.yaw, self.pitch, 0)
        
        forward  = rotate_vec(v(1,0,0), rot_matrix)
        left     = rotate_vec(v(0,1,0), rot_matrix)
        local_up = rotate_vec(v(0,0,1), rot_matrix)
        
        keys = pygame.key.get_pressed()
        
        speed = 50 if keys[pygame.K_LSHIFT] else 5
        speed *= (.1 if keys[pygame.K_LCTRL] else 1)
        
        if keys[pygame.K_w]: self.pos += forward*dt*speed
        if keys[pygame.K_s]: self.pos += -forward*dt*speed
        if keys[pygame.K_a]: self.pos += left*dt*speed
        if keys[pygame.K_d]: self.pos += -left*dt*speed
        if keys[pygame.K_SPACE]: self.pos += v(0, 0, 1)*dt*speed
        if keys[pygame.K_c]: self.pos += v(0, 0, -1)*dt*speed
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        perspective(self.fovy, self.display.get_width()/self.display.get_height(), 0.1)
        
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # rotates into the FLU coordinate system
        glMultMatrixf([
            [ 0., 0.,-1., 0.],
            [-1., 0., 0., 0.],
            [ 0., 1., 0., 0.],
            [ 0., 0., 0., 1.]
        ])
        # after that, +x is forward, +y is left, and +z is up
        
        glMultMatrixf(rot_matrix.T)
        
        glTranslate(*-self.pos)
        pos = self.pos
        
        self.world.draw()
        
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        with GLMatrix:
            glTranslate(*self.pos + forward + left - local_up)
            glBegin(GL_LINES)
            for i in xrange(3):
                glColor3f(*(j==i for j in xrange(3)))
                glVertex3f(0, 0, 0)
                glVertex3f(*(.1*(j==i) for j in xrange(3)))
            glEnd()
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        
        pygame.display.flip()

import ode

class Capsules(object):
    def __init__(self, world, space, pos, color, capsules, radius, mass=2, fixed=False, orientation=v(1, 0, 0, 0)):
        "capsules is a list of (start, end) points"
        
        self.capsules = capsules
        
        self.body = ode.Body(world)
        self.body.setPosition(pos)
        self.body.setQuaternion(orientation)
        m = ode.Mass()
        # computing MOI assuming sphere with .5 m radius
        m.setSphere(mass/(4/3*math.pi*.5**3), .5) # setSphereTotal is broken
        self.body.setMass(m)
        
        self.geoms = []
        self.geoms2 = []
        for start, end in capsules:
            self.geoms.append(ode.GeomTransform(space))
            x = ode.GeomCapsule(None, radius, (end-start).mag())
            self.geoms2.append(x)
            self.geoms[-1].setGeom(x)
            self.geoms[-1].setBody(self.body)
            x.setPosition((start+end)/2 + v(random.gauss(0, .01), random.gauss(0, .01), random.gauss(0, .01)))
            a = (end - start).unit()
            b = v(0, 0, 1)
            x.setQuaternion(sim_math_helpers.axisangle_to_quat((a%b).unit(), -math.acos(a*b)))
        
        self.color = color
        self.radius = radius
        
        if fixed:
            self.joint = ode.FixedJoint(world)
            self.joint.attach(self.body, None)
            self.joint.setFixed()
    
    def draw(self):
        q = gluNewQuadric()
        glColor3f(*self.color)
        with GLMatrix:
            rotate_to_body(self.body)
            for start, end in self.capsules:
                with GLMatrix:
                    glTranslate(*start)
                    gluSphere(q, self.radius, 30, 15)
                with GLMatrix:
                    glTranslate(*end)
                    gluSphere(q, self.radius, 30, 15)
                with GLMatrix:
                    glTranslate(*start)
                    a = (end - start).unit()
                    b = v(0, 0, 1)
                    glRotate(-math.degrees(math.acos(a*b)), *(a%b).unit())
                    gluCylinder(q, self.radius, self.radius, (end - start).mag(), 10, 1)

class Texture(object):
    def __init__(self, img, mipmap=True):
        import pygame
        data = pygame.image.tostring(img, "RGBA", True)
        #print len(data), img.get_height(), img.get_width()
        self.t = glGenTextures(1)
        with self:
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR if mipmap else GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 4)
            #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
            #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
            if mipmap:
                gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGBA, img.get_width(), img.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, data)
            else:
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.get_width(), img.get_height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, data)
    
    def __enter__(self):
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.t)
    
    def __exit__(self, *args):
        glDisable(GL_TEXTURE_2D)
    
    def __del__(self):
        glDeleteTextures(self.t)
    
if __name__ == '__main__':
    w = World()
    i = Interface()
    i.init(w)
    while True:
        i.step()
