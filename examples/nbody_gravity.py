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
    description = Label('mouse drag to rotate',
        anchor_x='center', x=window.width/2, y=10, bold=True, color=(255,255,255,255))
    tilt = 0
    rotate = 0
    
    @window.event
    def on_mouse_drag(x, y, dx, dy, button, modifiers):
        global rotate, tilt
        rotate += dx
        tilt += dy

    world = World()
    world.gravity = 0, 0, 0
    randpos = lambda size: [random() * size - size/2 for _ in range(3)]
      
    bodies = []
    for _ in range(30):
        position = randpos(200)
        size = random() * 5 + 1
        mass = size**3
        body = world.add_box_body(position=position, size=[size]*3, mass=mass)
        body.disable_deactivation()
        body.add_impulse(linear=randpos(20*mass))
        bodies.append(body)

    @window.event
    def on_draw():
        window.clear()
        with nested(Projection(0, 0, window.width, window.height, far=1000.0), Matrix, Lighting):
            glTranslatef(0, 0, -500)
            glRotatef(tilt*0.3, 1.0, 0, 0)
            glRotatef(rotate*0.3, 0.0, 1.0, 0)

            for body in bodies:
                with Matrix:
                    glMultMatrixf(body.matrix)
                    cube(size=body.size, color=(0.5, 0.5, 0.5, 1.0))

        fps.draw()
        description.draw()

    keys = pyglet.window.key.KeyStateHandler()
    window.push_handlers(keys)

    constant = 20.0
    def simulate(delta):
        for i, body1 in enumerate(bodies):
            for body2 in bodies[i+1:]:
                vec = body1.position - body2.position
                gravity = (body1.mass*body2.mass/vec.magnitude) * constant
                normal = vec.normalized
                body1.add_force(linear=normal.inversed*gravity, relative=False)
                body2.add_force(linear=normal*gravity, relative=False)

        world.step(delta, iterations=10)
    schedule_interval(simulate, 0.005)
    
    run()
