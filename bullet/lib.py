from ctypes import cdll
from sys import platform
import os

def resource(module, *path):
    here = os.path.dirname(
        os.path.abspath(module)
    )
    return os.path.join(here, *path)

def load(name):
    return cdll.LoadLibrary(
        resource(__file__, 'bin', name)
    )

lib = load('bullet.so')
