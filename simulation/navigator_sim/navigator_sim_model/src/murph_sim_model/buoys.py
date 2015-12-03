from murph_sim_rendering import sim_rendering_helpers as srh
from OpenGL.GL import *
from OpenGL.GLU import *

class Buoys(object):
    def __init__(self, pos,color,size, world, space):
       #global buoy_array
       #self.buoys = [(pos + [random.gauss(0, 5), random.gauss(0, 5), 0], color) for i in xrange(10) for color in [(1, 0, 0), (0, 1, 0),(0,0,1)]]
       #self.buoys = [(pos + [i*3,0,0],(i,1-i,0)) for i in xrange(2)]
       self.buoys = [pos,color,size] 

       self.geom = ode.GeomSphere(space, size)
       self.geom.setPosition(pos)

    def draw(self):
        q = gluNewQuadric()
        #for pos, color, size in self.buoys:
        with srh.GLMatrix:
            glTranslate(*self.buoys[0])
            glColor3f(*self.buoys[1])
            gluSphere(q, self.buoys[2], 30, 15)