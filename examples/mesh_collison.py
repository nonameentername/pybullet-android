from __future__ import with_statement

from random import random

from gletools import Projection, Matrix

from pyglet.window import Window, key
from pyglet.app import run
from pyglet.clock import schedule_interval, ClockDisplay
from pyglet.gl import *
from pyglet.text import Label

from bullet import World
    
def quad(normal, v1, v2, v3, v4):
    glNormal3f(*normal)
    glVertex3f(*v1)
    glVertex3f(*v2)
    glVertex3f(*v3)
    glVertex3f(*v4)

class Bunny(object):
    def __init__(self, world):
        v3f = [float(c)*0.2 for c in open('vertices').read().strip().split()]
        n3f = map(float, open('normals').read().strip().split())
        faces = map(int, open('faces').read().strip().split())
        self.display = pyglet.graphics.vertex_list_indexed(len(v3f)/3, faces,
            ('v3f', v3f),
            ('n3f', n3f),
        )

        self.shape = world.mesh_shape(faces, v3f)
        self.body = world.add_body(0, -16, 0, shape=self.shape, mass=0)

    def draw(self):
        with Matrix:
            glMultMatrixf(self.body.matrix)
            self.display.draw(GL_TRIANGLES)

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

if __name__ == '__main__':
    config = Config(buffers=2, samples=4)
    window = Window(fullscreen=True, vsync=False, config=config)
    gl_init()
    fps = ClockDisplay()
    description = Label('mouse drag to rotate, space to create more cubes',
        anchor_x='center', x=window.width/2, y=10, bold=True, color=(255,255,255,255))
    tilt = 0
    rotate = 0

    world = World()
    world.gravity = 0, -10, 0

    ground_size = (50, 25, 50)
    box_size = (2,2,2)

    bunny = Bunny(world)
    
    ground = world.add_box_body(position=(0,-56,0), size=ground_size)

    boxes = list()
    def add_box():
        x = random() * 10 - 5
        z = random() * 10 - 5
        boxes.append(
            world.add_box_body(position=(x,10,z), size=box_size, mass=5)
        )
    add_box()

    @window.event
    def on_draw():
        window.clear()
        with Projection(0, 0, window.width, window.height, far=1000.0):
            glEnable(GL_LIGHTING)
            glPushMatrix()
            glTranslatef(0, 15, -60)
            glRotatef(tilt*0.3, 1.0, 0, 0)
            glRotatef(rotate*0.3, 0.0, 1.0, 0)

            for box in boxes:
                with Matrix:
                    glMultMatrixf(box.matrix)
                    cube(size=box_size, color=(0.2, 0.2, 0.2, 1.0))
            
            with Matrix:
                glMultMatrixf(ground.matrix)
                cube(size=ground_size, color=(0.2, 0.7, 0.2, 1.0))

            bunny.draw()

            glPopMatrix()
            glDisable(GL_LIGHTING)

        fps.draw()
        description.draw()

    @window.event
    def on_mouse_drag(x, y, dx, dy, button, modifiers):
        global rotate, tilt
        rotate += dx
        tilt += dy

    @window.event
    def on_key_press(symbol, modifiers):
        if symbol == key.SPACE:
            add_box()

    def simulate(delta):
        world.step(delta, iterations=10)
    schedule_interval(simulate, 0.005)
    
    run()
