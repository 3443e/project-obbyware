#pragma once
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <raylib.h>
#include "OWPhysics/BigBoyConstants.hpp"
#include <array>

namespace OWPhysics {
    enum class CharacterState {
        Running,
        FreeFall,
        Jumping,
        Landed,
        Dead
    };

    struct OWLimb {
        btRigidBody* Body;
        btBoxShape* Shape;
        Vector3 Offset;   // offset from root in local space
        Vector3 Size;     // full size for drawing
    };

    class OWCharacter {
    public:
        btPairCachingGhostObject* Ghost;
        btCapsuleShape* CapsuleShape;
        btDiscreteDynamicsWorld* World;

        CharacterState State = CharacterState::FreeFall;
        bool OnGround = false;
        bool JumpRequested = false;
        btVector3 Velocity = btVector3(0, 0, 0);

        // limbs: 0=Torso 1=Head 2=LeftArm 3=RightArm 4=LeftLeg 5=RightLeg
        std::array<OWLimb, 6> Limbs;

        OWCharacter(btDiscreteDynamicsWorld* World, float PosX, float PosY, float PosZ);
        ~OWCharacter();

        void Update(float DeltaTime, Vector3 MoveDir);
        Vector3 GetPosition() const;
        Vector3 GetLimbPosition(int Index) const;

    private:
        bool CheckGround();
        void ApplyGravity(float DeltaTime);
        void ApplyMovement(Vector3 MoveDir);
        void DoJump();
        void SyncLimbs(); // moves limbs to follow ghost
        OWLimb CreateLimb(float SizeX, float SizeY, float SizeZ, float OffX, float OffY, float OffZ);
    };

} // namespace OWPhysics