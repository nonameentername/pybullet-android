#include "btBulletDynamicsCommon.h"
#include <stdio.h>

extern "C"{
    // rigid body info
    void * NewRigidBodyInfo(
        float mass,
        btDefaultMotionState* motion_state,
        btBoxShape* shape,
        btVector3* inertia
    ){
        return new btRigidBody::btRigidBodyConstructionInfo(
            mass,
            motion_state,
            shape,
            *inertia
        );
    }
    
    void DeleteRigidBodyInfo(btRigidBody::btRigidBodyConstructionInfo* info){
        delete info;
    }

    // rigid body
    void * NewRigidBody(btRigidBody::btRigidBodyConstructionInfo* info){
        return new btRigidBody(*info);
    }

    void DeleteRigidBody(btRigidBody* body){
        delete body;
    }

    void * RigidBodyGetMotionState(btRigidBody* body){
        return body->getMotionState();
    }

    void RigidBodyGetQuaternion(btRigidBody* body, btQuaternion* quaternion){
        *quaternion = body->getOrientation();
    }

    // shape
    void * NewBoxShape(float x, float y, float z){
        return new btBoxShape(btVector3(x, y, z));
    }

    void DeleteCollisionShape(btCollisionShape* shape){
        delete shape;
    }

    void BoxShapeCalculateInertia(btBoxShape* shape, float mass, btVector3* inertia){
        shape->calculateLocalInertia(mass,*inertia);
    }

    //transform
    void * NewTransform(){
        btTransform * transform = new btTransform();
        return transform;
    }

    void DeleteTransform(btTransform* transform){
        delete transform;
    }

    void TransformSetIdentity(btTransform* transform){
        transform->setIdentity();
    }

    void TransformSetOrigin(btTransform* transform, float x, float y, float z){
        transform->setOrigin(btVector3(x, y, z));
    }

    void TransformGetOrigin(btTransform* transform, btVector3* vector){
        *vector = transform->getOrigin();
    }

    void TransformGetOpengGLMatrix(btTransform* transform, btScalar* matrix){
        transform->getOpenGLMatrix(matrix);
    }

    // motionstate
    void * NewMotionState(btTransform* transform){
        return new btDefaultMotionState(*transform);
    }

    void DeleteMotionState(btDefaultMotionState* motion_state){
        delete motion_state;
    }

    void MotionStateGetWorldTransform(btDefaultMotionState* motion_state, btTransform* transform){
        motion_state->getWorldTransform(*transform);
    }

    // world
    void * NewDiscreteDynamicsWorld(
        btCollisionDispatcher* dispatcher,
        btAxisSweep3* overlapping_pair_cache,
        btSequentialImpulseConstraintSolver* solver,
        btDefaultCollisionConfiguration* collision_config
    ){
        return new btDiscreteDynamicsWorld(dispatcher, overlapping_pair_cache, solver, collision_config);
    }

    void DeleteDiscreteDynamicsWorld(btDiscreteDynamicsWorld* world){
        delete world;
    }

    void WorldSetGravity(btDiscreteDynamicsWorld* world, btVector3* vector){
        world->setGravity(*vector); 
    }

    void WorldAddBody(btDiscreteDynamicsWorld* world, btRigidBody* body){
        world->addRigidBody(body);
    }

    void WorldStepSimulation(btDiscreteDynamicsWorld* world, float delta, int iterations){
        world->stepSimulation(delta, iterations);
    }

    // collision config
    void* NewDefaultCollisionConfiguration(){
        return new btDefaultCollisionConfiguration();
    }

    void DeleteDefaultCollisionConfiguration(btDefaultCollisionConfiguration* collision){
        delete collision;
    }

    // collision dispatcher
    void* NewCollisionDispatcher(btDefaultCollisionConfiguration* collision_configuration){
        return new btCollisionDispatcher(collision_configuration);
    }

    void DeleteCollisionDispatcher(btCollisionDispatcher* collision_dispatcher){
        delete collision_dispatcher;
    }

    // axis sweep
    void* NewAxisSweep3(btVector3* abb_min, btVector3* abb_max, int proxies){
        return new btAxisSweep3(*abb_min, *abb_max, proxies);
    }

    void DeleteAxisSweep3(btAxisSweep3* axis_sweep){
        delete axis_sweep;
    }

    // constriant solver
    void * NewSequentialImpulseConstraintSolver(){
        return new btSequentialImpulseConstraintSolver;
    }

    void DeleteSequentialImpulseConstraintSolver(btSequentialImpulseConstraintSolver* solver){
        delete solver;
    }
}
