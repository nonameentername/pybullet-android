from __future__ import with_statement

from random import random
from contextlib import nested

from gletools import Projection, Matrix, Lighting

from pyglet.window import Window, key
from pyglet.app import run
from pyglet.clock import schedule_interval, ClockDisplay
from pyglet.gl import *
from pyglet.text import Label

from bullet import World
from common import cube, gl_init
    
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
        with nested(Projection(0, 0, window.width, window.height, far=1000.0), Matrix, Lighting):
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
