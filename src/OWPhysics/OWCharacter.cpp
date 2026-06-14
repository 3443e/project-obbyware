#include "OWPhysics/OWCharacter.hpp"
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

namespace OWPhysics {
    OWLimb OWCharacter::CreateLimb(float SizeX, float SizeY, float SizeZ, float OffX, float OffY, float OffZ) {
        OWLimb Limb;
        Limb.Size = { SizeX, SizeY, SizeZ };
        Limb.Offset = { OffX,  OffY,  OffZ  };

        Limb.Shape = new btBoxShape(btVector3(0.5f * SizeX, 0.5f * SizeY, 0.5f * SizeZ));
        Limb.Shape->setMargin(OWConstants::CollisionMargin);

        btTransform T;
        T.setIdentity();

        btDefaultMotionState* MS = new btDefaultMotionState(T);
        btRigidBody::btRigidBodyConstructionInfo Info(0, MS, Limb.Shape);
        Limb.Body = new btRigidBody(Info);
        Limb.Body->setCollisionFlags(Limb.Body->getCollisionFlags() |btCollisionObject::CF_KINEMATIC_OBJECT);
        Limb.Body->setActivationState(DISABLE_DEACTIVATION);

        World->addRigidBody(Limb.Body, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter);

        return Limb;
    }

    OWCharacter::OWCharacter(btDiscreteDynamicsWorld* InWorld, float PosX, float PosY, float PosZ) : World(InWorld) {
        CapsuleShape = new btCapsuleShape(1.0f, 2.0f);
        CapsuleShape->setMargin(OWConstants::CollisionMargin);

        Ghost = new btPairCachingGhostObject();
        btTransform Transform;
        Transform.setIdentity();
        Transform.setOrigin(btVector3(PosX, PosY, PosZ));
        Ghost->setWorldTransform(Transform);
        Ghost->setCollisionShape(CapsuleShape);
        Ghost->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
        World->addCollisionObject(Ghost, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter);

        // R6 limbs sizes and offsets
        //                    SizeX  SizeY  SizeZ   OffX   OffY   OffZ
        Limbs[0] = CreateLimb(2.0f,  2.0f,  1.0f,   0.0f,  0.0f,  0.0f); // Torso
        Limbs[1] = CreateLimb(1.0f,  1.0f,  1.0f,   0.0f,  1.5f,  0.0f); // Head
        Limbs[2] = CreateLimb(1.0f,  2.0f,  1.0f,  -1.5f,  0.0f,  0.0f); // Left Arm
        Limbs[3] = CreateLimb(1.0f,  2.0f,  1.0f,   1.5f,  0.0f,  0.0f); // Right Arm
        Limbs[4] = CreateLimb(1.0f,  2.0f,  1.0f,  -0.5f, -2.0f,  0.0f); // Left Leg
        Limbs[5] = CreateLimb(1.0f,  2.0f,  1.0f,   0.5f, -2.0f,  0.0f); // Right Leg
    }


    OWCharacter::~OWCharacter() {
        for (auto& Limb : Limbs) {
            World->removeRigidBody(Limb.Body);
            delete Limb.Body->getMotionState();
            delete Limb.Body;
            delete Limb.Shape;
        }
        World->removeCollisionObject(Ghost);
        delete Ghost;
        delete CapsuleShape;
    }

    void OWCharacter::SyncLimbs() {
        btVector3 Root = Ghost->getWorldTransform().getOrigin();
        for (auto& Limb : Limbs) {
            btTransform T;
            T.setIdentity();
            T.setOrigin(Root + btVector3(Limb.Offset.x, Limb.Offset.y, Limb.Offset.z));
            Limb.Body->getMotionState()->setWorldTransform(T);
            Limb.Body->setWorldTransform(T);
        }
    }

    Vector3 OWCharacter::GetLimbPosition(int Index) const {
        btVector3 Pos = Limbs[Index].Body->getWorldTransform().getOrigin();
        return { Pos.x(), Pos.y(), Pos.z() };
    }


    bool OWCharacter::CheckGround() {
        btTransform T = Ghost->getWorldTransform();
        btVector3 From = T.getOrigin();
        btVector3 To = From - btVector3(0, 2.2f, 0);

        btCollisionWorld::ClosestRayResultCallback Ray(From, To);
        Ray.m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
        World->rayTest(From, To, Ray);

        return Ray.hasHit();
    }

    void OWCharacter::ApplyGravity(float DeltaTime) {
        if (!OnGround) {
            Velocity.setY(Velocity.y() + OWConstants::Gravity * DeltaTime);
        } else {
            if (Velocity.y() < 0.0f) {
                Velocity.setY(0.0f);
            }
        }
    }

    void OWCharacter::ApplyMovement(Vector3 MoveDir) {
        Velocity.setX(MoveDir.x * OWConstants::WalkSpeed);
        Velocity.setZ(MoveDir.z * OWConstants::WalkSpeed);
    }

    void OWCharacter::DoJump() {
        //  kJumpVelocityGrid() = 50.0f
        Velocity.setY(OWConstants::JumpVelocity);
        State = CharacterState::Jumping;
        OnGround = false;
    }

    void OWCharacter::Update(float DeltaTime, Vector3 MoveDir) {
        OnGround = CheckGround();

        switch (State) {
            case CharacterState::Running:
                if (!OnGround) State = CharacterState::FreeFall;
                break;
            case CharacterState::FreeFall:
                if (OnGround)  State = CharacterState::Landed;
                break;
            case CharacterState::Jumping:
                if (Velocity.y() <= 0.0f) State = CharacterState::FreeFall;
                break;
            case CharacterState::Landed:
                State = CharacterState::Running;
                break;
            case CharacterState::Dead:
                break;
        }

        if (JumpRequested && OnGround) {
            DoJump();
            JumpRequested = false;
        }

        ApplyGravity(DeltaTime);
        ApplyMovement(MoveDir);

        btVector3 Pos = Ghost->getWorldTransform().getOrigin();
        if (Pos.y() < OWConstants::FallDestroyHeight) {
            State = CharacterState::Dead;
        }

        btTransform T = Ghost->getWorldTransform();
        T.setOrigin(T.getOrigin() + Velocity * DeltaTime);
        Ghost->setWorldTransform(T);

        SyncLimbs(); // keep limbs glued to root
    }
    Vector3 OWCharacter::GetPosition() const {
        btVector3 Pos = Ghost->getWorldTransform().getOrigin();
        return {Pos.x(), Pos.y(), Pos.z() };
    }

} // namespace OWPhysics