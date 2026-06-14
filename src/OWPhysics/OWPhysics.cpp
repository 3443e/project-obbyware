#include "OWPhysics/OWPhysics.hpp"

namespace OWPhysics {
    Physics::Physics() {
        CollisionConfig = new btDefaultCollisionConfiguration();
        Dispatcher = new btCollisionDispatcher(CollisionConfig);
        Broadphase = new btDbvtBroadphase();
        Solver = new btSequentialImpulseConstraintSolver();
        World = new btDiscreteDynamicsWorld(Dispatcher, Broadphase, Solver, CollisionConfig);

        World->setGravity(btVector3(0, OWConstants::Gravity, 0));
    }

    Physics::~Physics() {
        delete World;
        delete Solver;
        delete Broadphase;
        delete Dispatcher;
        delete CollisionConfig;
    }
    
    void Physics::Step(float DeltaTime) {
        Accumulator += DeltaTime;
        while (Accumulator >= OWConstants::FixedStep) {
            World->stepSimulation(OWConstants::FixedStep, 1, OWConstants::FixedStep);
            Accumulator -= OWConstants::FixedStep;
        }
    }

    btRigidBody* Physics::AddBox(float X, float Y, float Z, float PosX, float PosY, float PosZ) {
        btBoxShape* Shape = new btBoxShape(
            btVector3(0.5f * X, 0.5f * Y, 0.5f * Z)
        );
        Shape->setMargin(OWConstants::CollisionMargin);

        btTransform Transform;
        Transform.setIdentity();
        Transform.setOrigin(btVector3(PosX, PosY, PosZ));

        btDefaultMotionState* MotionState = new btDefaultMotionState(Transform);

        // 
        btRigidBody::btRigidBodyConstructionInfo Info(0, MotionState, Shape); /* mass 0 = static */
        btRigidBody* Body = new btRigidBody(Info);

        Body->setCollisionFlags(Body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);

        World->addRigidBody(Body);
        return Body;
    }

} // namespace OWPhysics