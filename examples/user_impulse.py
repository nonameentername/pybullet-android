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

    @window.event
    def on_key_press(symbol, modifiers):
        if symbol == key.UP:
            box.add_rel_impulse(impulse=(0,5,0))
        elif symbol == key.DOWN:
            box.add_rel_impulse(impulse=(0,-5,0))
        elif symbol == key.LEFT:
            box.add_rel_impulse(impulse=(-5,0,0))
        elif symbol == key.RIGHT:
            box.add_rel_impulse(impulse=(5,0,0))
        elif symbol == key.DELETE:
            box.add_rel_torque_impulse(torque=(0,0,5))
        elif symbol == key.PAGEDOWN:
            box.add_rel_torque_impulse(torque=(0,0,-5))
        elif symbol == key.HOME:
            box.add_rel_torque_impulse(torque=(-5,0,0))
        elif symbol == key.END:
            box.add_rel_torque_impulse(torque=(5,0,0))

    def simulate(delta):
        world.step(delta, iterations=10)
    schedule_interval(simulate, 0.005)
    
    run()
