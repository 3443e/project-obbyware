#pragma once

namespace OWPhysics {
    namespace OWConstants {
        // bullet
        constexpr float CollisionMargin = 0.05f;

        // owphysics
        constexpr float Gravity = -196.2f;     // studs/s²
        constexpr float FixedStep =  1.0f/240.0f; // worldDt
        constexpr int StepsPerSecond =  240;

        // character
        constexpr float WalkSpeed = 16.0f; // studs/s
        constexpr float JumpVelocity = 50.0f; // studs/s upward impulse
        constexpr float HipHeight = 2.0f; // capsule offset from floor
        constexpr float MaxSlopeAngle = 89.0f; // degrees

        // air control
        constexpr float AirTurnSpeed = 6.0f;
        constexpr float FloorVelInfluence = 0.95f; // momentum kept from floor on leave
        constexpr float AirVelocityDecay = 0.05f; // per 1/30s

        // Death idk
        constexpr float FallDestroyHeight = -500.0f; // from World constructor
    }
}