from __future__ import with_statement

from random import random

from gletools import Projection, Matrix, Lighting
from contextlib import nested

from pyglet.window import Window, key
from pyglet.app import run
from pyglet.clock import schedule_interval, ClockDisplay
from pyglet.gl import *
from pyglet.text import Label

from bullet import World

from common import cube, gl_init
    
if __name__ == '__main__':
    config = Config(buffers=2, samples=4)
    window = Window(fullscreen=True, vsync=False, config=config)
    gl_init()
    fps = ClockDisplay()
    description = Label('cursor keys for movement, delete/page down/home/end for rotate',
        anchor_x='center', x=window.width/2, y=10, bold=True, color=(255,255,255,255))

    world = World()
    world.gravity = 0, 0, 0
       
    box = world.add_box_body(position=(0,0,0), size=(10,10,10), mass=5)
    box.disable_deactivation()

    @window.event
    def on_draw():
        window.clear()
        with nested(Projection(0, 0, window.width, window.height, far=1000.0), Matrix, Lighting):
            glTranslatef(0, 0, -40)

            with Matrix:
                glMultMatrixf(box.matrix)
                cube(size=(10,10,10), color=(0.5, 0.5, 0.5, 1.0))

        fps.draw()
        description.draw()

    keys = pyglet.window.key.KeyStateHandler()
    window.push_handlers(keys)

    def simulate(delta):
        f = 5000*delta
        if keys[key.UP]:
            box.add_force(linear=(0,f,0))
        if keys[key.DOWN]:
            box.add_force(linear=(0,-f,0))
        if keys[key.LEFT]:
            box.add_force(linear=(-f,0,0))
        if keys[key.RIGHT]:
            box.add_force(linear=(f,0,0))
        if keys[key.DELETE]:
            box.add_force(torque=(0,0,f))
        if keys[key.PAGEDOWN]:
            box.add_force(torque=(0,0,-f))
        if keys[key.HOME]:
            box.add_force(torque=(-f,0,0))
        if keys[key.PAGEDOWN]:
            box.add_force(torque=(f,0,0))

        world.step(delta, iterations=10)
    schedule_interval(simulate, 0.005)
    
    run()
