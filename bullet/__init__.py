from ctypes import cdll, c_void_p, c_float, c_int, c_byte, Structure, byref, pointer
from .util import resource
from .math3d import Matrix

lib = cdll.LoadLibrary(
    resource(__file__, '_bullet.so')
)

def types(fun, restype, *argtypes):
    fun.restype = restype
    fun.argtypes = argtypes
    return fun

class BulletObject(object):
    def __init__(self, *args):
        self.handle = self.new(*args)

    def __del__(self):
        self.delete(self.handle)

class Vector(Structure):
    _fields_ = [
        ('x', c_float),
        ('y', c_float),
        ('z', c_float),
        ('w', c_float),
    ]
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    def __iter__(self):
        return iter((self.x, self.y, self.z))

class Quaternion(Structure):
    _fields_ = [
        ('x', c_float),
        ('y', c_float),
        ('z', c_float),
        ('w', c_float),
    ]

    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

    def __iter__(self):
        return iter((self.x, self.y, self.z, self.w))


class CollisionConfiguration(BulletObject):
    new = lib.NewDefaultCollisionConfiguration
    delete = lib.DeleteDefaultCollisionConfiguration

class CollisionDispatcher(BulletObject):
    new = lib.NewCollisionDispatcher
    delete = lib.DeleteCollisionDispatcher

    def __init__(self, config):
        BulletObject.__init__(self, config.handle)


class AxisSweep(BulletObject):
    new = lib.NewAxisSweep3
    delete = lib.DeleteAxisSweep3

    def __init__(self, abb_min, abb_max, proxies):
        BulletObject.__init__(self, byref(abb_min), byref(abb_max), proxies)

class ConstraintSolver(BulletObject):
    new = lib.NewSequentialImpulseConstraintSolver
    delete = lib.DeleteSequentialImpulseConstraintSolver

class World(BulletObject):
    new = lib.NewDiscreteDynamicsWorld
    delete = lib.DeleteDiscreteDynamicsWorld
    types(lib.WorldStepSimulation, None, c_void_p, c_float, c_int)

    def __init__(self):
        self.collision_config = CollisionConfiguration()
        self.dispatcher = CollisionDispatcher(self.collision_config)
        self.overlapping_pair_cache = AxisSweep(
            abb_min = Vector(-10000, -10000, -1000),
            abb_max = Vector(10000, 10000, 10000),
            proxies = 1024,
        )
        self.solver = ConstraintSolver()
        BulletObject.__init__(self,
            self.dispatcher.handle,
            self.overlapping_pair_cache.handle,
            self.solver.handle,
            self.collision_config.handle,
        )
        self.bodies = []
    
    def set_gravity(self, (x, y, z)):
        vector = Vector(x, y, z)
        lib.WorldSetGravity(self.handle, byref(vector))
    gravity = property(None, set_gravity)
    del set_gravity

    def add(self, body):
        self.bodies.append(body)
        lib.WorldAddBody(self.handle, body.handle)

    def step(self, delta, iterations=10):
        lib.WorldStepSimulation(self.handle, delta, iterations) 

    def box_shape(self, width, height, length):
        return BoxShape(width, height, length)

    def mesh_shape(self, faces, vertices, compress=True, build_bhv=True):
        return MeshShape(faces, vertices, compress, build_bhv)

    def add_body(self, x, y, z, shape, mass=0):
        transform = Transform(x, y, z)
        motion_state = MotionState(transform)
        inertia = Vector(0, 0, 0)
        if mass > 0:
            shape.calculate_inertia(mass, inertia)
        body_info = RigidBodyInfo(
            mass = mass,
            motion_state = motion_state,
            shape = shape,
            inertia = inertia,
        )
        body = RigidBody(body_info)
        self.add(body)
        return body

    def add_box_body(self, position, size, mass=0):
        x, y, z = position
        shape = self.box_shape(*size)
        return self.add_body(x, y, z, shape, mass)

class BoxShape(BulletObject):
    new = types(lib.NewBoxShape, c_void_p, c_float, c_float, c_float)
    delete = lib.DeleteCollisionShape
    types(lib.BoxShapeCalculateInertia, None, c_void_p, c_float, c_void_p)

    def __init__(self, width, height, length):
        BulletObject.__init__(self, width, height, length)

    def calculate_inertia(self, mass, inertia):
        lib.BoxShapeCalculateInertia(self.handle, mass, byref(inertia))

class MeshShape(BulletObject):
    new = types(lib.NewTriangleMeshShape, c_void_p, c_void_p, c_byte, c_byte)
    delete = lib.DeleteTriangleMeshShape

    def __init__(self, faces, vertices, compress, build_bhv):
        face_count = len(faces)/3
        vertex_count = len(vertices)/3

        self.faces = (c_int*len(faces))(*faces)
        self.vertices = (c_float*len(vertices))(*vertices)
        self.array = lib.NewTriangleIndexVertexArray(face_count, self.faces, vertex_count, self.vertices)
        BulletObject.__init__(self, self.array, int(compress), int(build_bhv))

    def __del__(self):
        lib.DeleteTriangleIndexVertexArray(self.array) 
        lib.DeleteTriangleMeshShape(self.handle)

class Transform(BulletObject):
    new = lib.NewTransform
    delete = lib.DeleteTransform
    types(lib.TransformSetOrigin, None, c_void_p, c_float, c_float, c_float)

    def __init__(self, x, y, z):
        BulletObject.__init__(self)
        self.set_identity()
        self.origin = x, y, z

    def set_identity(self):
        lib.TransformSetIdentity(self.handle)

    def get_origin(self):
        vector = Vector(0,0,0)
        lib.TransformGetOrigin(self.handle, byref(vector))
        return vector
    def set_origin(self, (x, y, z)):
        lib.TransformSetOrigin(self.handle, x, y, z)
    origin = property(get_origin, set_origin)
    del get_origin, set_origin
   
    @property
    def matrix(self):
        matrix = (c_float*16)()
        lib.TransformGetOpengGLMatrix(self.handle, matrix)
        return matrix

class MotionState(BulletObject):
    new = lib.NewMotionState
    delete = lib.DeleteMotionState
    types(lib.MotionStateGetWorldTransform, None, c_void_p, c_void_p)

    def __init__(self, transform):
        self.transform = transform  
        BulletObject.__init__(self, transform.handle)

    @property
    def world_transform(self):
        transform = Transform(0, 0, 0)
        lib.MotionStateGetWorldTransform(self.handle, transform.handle)
        return transform

class RigidBodyInfo(BulletObject):
    new = types(lib.NewRigidBodyInfo, c_void_p, c_float, c_void_p, c_void_p, c_void_p)
    delete = lib.DeleteRigidBodyInfo

    def __init__(self, mass, motion_state, shape, inertia):
        self.motion_state = motion_state
        self.shape = shape
        self.inertia = inertia

        BulletObject.__init__(self,
            mass,
            motion_state.handle,
            shape.handle,
            byref(inertia),
        )

class RigidBody(BulletObject):
    new = lib.NewRigidBody
    delete = lib.DeleteRigidBody

    def __init__(self, info):
        self.info = info
        BulletObject.__init__(self, info.handle)

    @property
    def motion_state(self):
        return self.info.motion_state

    @property
    def position(self):
        return self.motion_state.world_transform.origin

    @property
    def quaternion(self):
        quaternion = Quaternion()
        lib.RigidBodyGetQuaternion(self.handle, byref(quaternion))
        return quaternion

    @property
    def matrix(self):
        return self.motion_state.world_transform.matrix
        '''
        matrix = Matrix()
        matrix.position = self.position
        matrix.quaternion = self.quaternion
        return matrix
        '''
