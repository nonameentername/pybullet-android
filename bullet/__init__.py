from math import sin, acos
from ctypes import cdll, c_void_p, c_float, c_int, c_byte, Structure, byref, pointer, CFUNCTYPE
from .lib import lib
from .math3d import Matrix

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
    types(lib.VectorInterpolate, None, c_void_p, c_void_p, c_void_p, c_float)
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

    def __sub__(self, other):
        return Vector(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z,
        )

    def interpolate(self, other, scalar):
        result = Vector()
        lib.VectorInterpolate(byref(self), byref(other), byref(result), scalar)
        return result

    @property
    def magnitude(self):
        return self.x**2 + self.y**2 + self.z**2

    @property
    def length(self):
        return self.magnitude**0.5

    @property
    def normalized(self):
        return self / self.length

    def __div__(self, scalar):
        return Vector(
            self.x / scalar,
            self.y / scalar,
            self.z / scalar,
        )

    def __mul__(self, scalar):
        return Vector(
            self.x * scalar,
            self.y * scalar,
            self.z * scalar,
        )

    @property
    def inversed(self):
        return self * -1
        

class Quaternion(Structure):
    types(lib.QuaternionInterpolate, None, c_void_p, c_void_p, c_void_p, c_float)

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

    '''
    #bullet quaternion interpolation does not work
    def interpolate(self, other, scalar):
        result = Quaternion()
        lib.QuaternionInterpolate(byref(self), byref(other), byref(result), scalar)
        return result
    '''
    
    def interpolate(self, other, t):
        tmp = Quaternion()

        cosom = self.x*other.x + self.y*other.y + self.z*other.z + self.w*other.w
        if cosom < 0:
            cosom = -cosom
            tmp.x = -other.x
            tmp.y = -other.y
            tmp.z = -other.z
            tmp.w = -other.w
        else:
            tmp.x = other.x
            tmp.y = other.y
            tmp.z = other.z
            tmp.w = other.w
    
        if 1.0 - cosom > 0.1:
            omega = acos(cosom)
            sinom = sin(omega)
            scale0 = sin((1.0-t) * omega) / sinom
            scale1 = sin(t*omega) / sinom
        else:
            scale0 = 1.0 - t
            scale1 = t

        return Quaternion(
            x = scale0 * self.x + scale1 * tmp.x,
            y = scale0 * self.y + scale1 * tmp.y,
            z = scale0 * self.z + scale1 * tmp.z,
            w = scale0 * self.w + scale1 * tmp.w,
        )

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
    types(lib.WorldSetTickCallback, None, c_void_p, c_void_p)

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
        body.world = self
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

    def remove(self, body):
        body.world = None
        lib.WorldRemoveBody(self.handle, body.handle)

    def add_box_body(self, position, size, mass=0):
        x, y, z = position
        shape = self.box_shape(*size)
        return self.add_body(x, y, z, shape, mass)

    def set_tick(self, callback):
        self._tick = CFUNCTYPE(None, c_float)(callback)
        lib.WorldSetTickCallback(self.handle, self._tick)
    tick = property(None, set_tick)
    del set_tick

class BoxShape(BulletObject):
    new = types(lib.NewBoxShape, c_void_p, c_float, c_float, c_float)
    delete = lib.DeleteCollisionShape
    types(lib.BoxShapeCalculateInertia, None, c_void_p, c_float, c_void_p)

    def __init__(self, width, height, length):
        self.width = width
        self.height = height
        self.length = length
        BulletObject.__init__(self, width, height, length)

    def calculate_inertia(self, mass, inertia):
        lib.BoxShapeCalculateInertia(self.handle, mass, byref(inertia))

    @property
    def size(self):
        return self.width, self.height, self.length

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
        self.mass = mass
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

    def remove(self):
        self.world.remove(self)

    @property
    def size(self):
        return self.info.shape.size 

    @property
    def mass(self):
        return self.info.mass

    @property
    def motion_state(self):
        return self.info.motion_state

    def get_position(self):
        position = Vector()
        lib.RigidBodyGetPosition(self.handle, byref(position))
        return position
    
    def set_position(self, position):
        position = Vector(*position)
        lib.RigidBodySetPosition(self.handle, byref(position))
    position = property(get_position, set_position)
    del get_position, set_position

    def get_quaternion(self):
        quaternion = Quaternion()
        lib.RigidBodyGetQuaternion(self.handle, byref(quaternion))
        return quaternion
    
    def set_quaternion(self, quaternion):
        quaternion = Quaternion(*quaternion)
        lib.RigidBodySetQuaternion(self.handle, byref(quaternion))
    quaternion = property(get_quaternion, set_quaternion)
    del get_quaternion, set_quaternion

    @property
    def matrix(self):
        return self.motion_state.world_transform.matrix
        '''
        matrix = Matrix()
        matrix.position = self.position
        matrix.quaternion = self.quaternion
        return matrix
        '''

    def add_force(self, linear=(0,0,0), torque=(0,0,0), relative=True):
        if relative:
            if linear != (0,0,0):
                linear = Vector(*linear)
                lib.RigidBodyAddRelForce(self.handle, byref(linear))
            if torque != (0,0,0):
                torque = Vector(*torque)
                lib.RigidBodyAddRelTorque(self.handle, byref(torque))
        else:
            if linear != (0,0,0):
                linear = Vector(*linear)
                lib.RigidBodyAddForce(self.handle, byref(linear))
            if torque != (0,0,0):
                torque = Vector(*torque)
                lib.RigidBodyAddTorque(self.handle, byref(torque))

    def add_impulse(self, linear=(0,0,0), torque=(0,0,0), relative=True):
        if relative:
            if linear != (0,0,0):
                linear = Vector(*linear)
                lib.RigidBodyAddRelImpulse(self.handle, byref(linear)) 
            if torque != (0,0,0):
                torque = Vector(*torque)
                lib.RigidBodyAddRelTorqueImpulse(self.handle, byref(torque)) 
        else:
            if linear != (0,0,0):
                linear = Vector(*linear)
                lib.RigidBodyAddImpulse(self.handle, byref(linear)) 
            if torque != (0,0,0):
                torque = Vector(*torque)
                lib.RigidBodyAddTorqueImpulse(self.handle, byref(torque)) 
    
    def disable_deactivation(self):
        lib.RigidBodyDisableDeactivation(self.handle)
