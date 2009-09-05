from pyglet.gl import *

def quad(normal, v1, v2, v3, v4):
    glNormal3f(*normal)
    glVertex3f(*v1)
    glVertex3f(*v2)
    glVertex3f(*v3)
    glVertex3f(*v4)

def cube(size, color=(1,1,1,1)):
    width, height, length = size
    c1 = -width, -height, -length 
    c2 = width, -height, -length
    c3 = width, height, -length 
    c4 = -width, height, -length
    
    c5 = -width, -height, length
    c6 = width, -height, length
    c7 = width, height, length
    c8 = -width, height, length

    front = 0, 0, -1
    back = 0, 0, 1
    left = -1, 0, 0
    right = 1, 0, 0
    top = 0, -1, 0 
    bottom = 0, 1, 0

    glColor4f(*color)
    glBegin(GL_QUADS)
    quad(front, c4, c3, c2, c1)
    quad(back, c5, c6, c7, c8)
    quad(left, c5, c8, c4, c1)
    quad(right, c3, c7, c6, c2)
    quad(top, c2, c6, c5, c1)
    quad(bottom, c4, c8, c7, c3)
    glEnd()

def gl_init():
    glEnable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHT0)
    glEnable(GL_COLOR_MATERIAL)
    glShadeModel(GL_SMOOTH)

    glLightfv(GL_LIGHT0, GL_AMBIENT, (GLfloat*4)(0.1, 0.1, 0.1, 0.2))
    glLightfv(GL_LIGHT0, GL_DIFFUSE, (GLfloat*4)(0.5, 0.5, 0.5, 0.8))
    glLightfv(GL_LIGHT0, GL_SPECULAR, (GLfloat*4)(0.3, 0.3, 0.3, 0.5))
    glLightfv(GL_LIGHT0, GL_POSITION, (GLfloat*4)(0, 0, 0, 1))

