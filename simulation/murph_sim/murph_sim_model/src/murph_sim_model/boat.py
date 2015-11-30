from murph_sim_rendering import sim_rendering_helpers as srh
from OpenGL.GL import *
from OpenGL.GLU import *

class Boat(object):
    is_base_link = True
    
    def __init__(self, body):
        self.body = body
        self.vectors = []
    
    def draw(self):
        with srh.GLMatrix:
            srh.rotate_to_body(self.body)
            
            q = gluNewQuadric()
            with srh.GLMatrix:
                glTranslate(+.2, 0, 0)
                glColor3f(0, 1, 0)
                gluSphere(q, 0.5, 40, 20)
            with srh.GLMatrix:
                glTranslate(-.2, 0, 0)
                glColor3f(1, 0, 0)
                gluSphere(q, 0.5, 40, 20)
            with srh.GLMatrix:
                glTranslate(+.4, -.2, +.3)
                glColor3f(0, 0, 0)
                gluSphere(q, 0.1, 40, 20)
            with srh.GLMatrix:
                glTranslate(+.4, +.2, +.3)
                glColor3f(0, 0, 0)
                gluSphere(q, 0.1, 40, 20)
            with srh.GLMatrix:
                glTranslate(0, 0, +.5)
                glColor3f(0, 1, 0)
                gluSphere(q, 0.1, 20, 10)
            
            glDisable(GL_DEPTH_TEST)
            glBegin(GL_LINES)
            for start, end in self.vectors:
                glColor3f(0, 0, 0)
                glVertex3f(*start)
                glColor3f(1, 1, 1)
                glVertex3f(*end)
            glEnd()
            glEnable(GL_DEPTH_TEST)