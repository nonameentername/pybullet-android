from ctypes import cdll
from sys import platform, maxint
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

if maxint == 0x7fffffffffffffff:
    arch = 'x64'
elif maxint == 0x7fffffff:
    arch = 'x86'

if arch == 'x86':
    if platform == 'linux2':
        lib = load('bullet-x86.so')
    elif platform == 'darwin':
        lib = load('bullet-x86.dylib')
    elif platform == 'windows':
        lib = load('bullet-x86.dll')
    else:
        raise ImportError('unsupported platform: %s' % platform)
elif arch == 'x64':
    if platform == 'linux2':
        lib = load('bullet-x64.so')
    elif platform == 'darwin':
        lib = load('bullet-x64.dylib')
    elif platform == 'windows':
        lib = load('bullet-x64.dll')
    else:
        raise ImportError('unsupported platform: %s' % platform)

