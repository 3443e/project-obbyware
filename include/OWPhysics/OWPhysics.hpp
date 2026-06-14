#pragma once
#include <btBulletDynamicsCommon.h>
#include "OWPhysics/BigBoyConstants.hpp"

namespace OWPhysics {
    class Physics {
    public:
        btDiscreteDynamicsWorld* World;
        btDefaultCollisionConfiguration* CollisionConfig;
        btCollisionDispatcher* Dispatcher;
        btDbvtBroadphase* Broadphase;
        btSequentialImpulseConstraintSolver* Solver;

        float Accumulator = 0.0f;

        Physics();
        ~Physics();

        void Step(float DeltaTime);

        btRigidBody* AddBox(float HalfX, float HalfY, float HalfZ, float X, float Y, float Z); // Mass 0 = static
    };
} // namespace OWPhysics