#pragma once
#include "OWInstance/OWPart.hpp"
#include "libowsolver/ConstraintTypes.hpp"
#include <cstddef>
#include <raylib.h>
#include <vector>
#include "OWCamera.hpp" 

class OWCamera;

// some formulas
// torque = -kP * Ibody * tilt - kD * Ibody * angVel
// yAccel = kAltitudeP * error - kAltitudeD * velError
// force = mass * yAccel * 0.2 (PGS scale)
// RunningBase.cpp:  torqueY = kTurnP * IbodyY * (desiredAngVelY - currentAngVelY), clamp ±1e5
// Freefall:   desiredAngVelY = ±8.0 if |error|>0.2, else 0.25*8*error
// Jumping: force = mass * kJumpP * (JumpPower - velY) * 0.52
// moveForce = mass * 150 * velError (PGS)
class OWPlayerController {
public:
    OWPlayerController(OWPart* part);
    ~OWPlayerController();

    void updateInput(float dt);
    void updateCamera() {};
    
    void setCamera(OWCamera* cam) { camera = cam; }
    
    void setJumpPower(float v) { jumpPower = v; }
    void setWalkSpeed(float v) { walkSpeed = v; }
    void setHipHeight(float h) { hipHeight = h; }

    // all of this is for the debug menu
    bool isGrounded() const { return grounded; } 
    bool isJumping() const { return jumping; }
    bool isHipEnabled() const { return hipEnabled; }
    float getJumpTimer() const { return jumpTimer; }
    glm::vec3 getDesiredVel() const { return cachedDesiredVel; }
    float getDesiredAngVelY() const { return cachedDesiredAngVelY; }
    float getFloorDist() const;
    bool isFacingLadder() const { return facingLadder; }
    void setDebugRaysEnabled(bool e) { debugRaysEnabled = e; }
    
    // call inside BeginMode3D
    void renderDebugRays();

    enum class State {
        Running,
        Jumping,
        Freefall,
        Landed, 
        Climbing
    };
    State getState() const { return state; }

private:
    OWPart* part;
    OWCamera* camera = nullptr;

    static constexpr float K_ALTITUDE_P = 30000.0f;
    static constexpr float K_ALTITUDE_D = 1100.0f;    

    static constexpr float K_BALANCE_P_GROUND = 2250.0f;   // GettingUp PGS
    static constexpr float K_BALANCE_P_AIR = 5000.0f;      // Freefall
    static constexpr float K_BALANCE_D = 50.0f;

    static constexpr float K_MOVE_P_PGS = 150.0f;
    static constexpr float MAX_GROUND_MOVE_ACCEL = 500.0f;
    static constexpr float MAX_AIR_MOVE_ACCEL = 143.0f;   
    static constexpr float ALTITUDE_SCALE = 0.2f;

    static constexpr float AUTO_TURN_SPEED = 8.0f;
    static constexpr float TURN_SLOSH_DEADZONE = 0.2f;     // ~5.7° 

    static constexpr float K_TURN_P_GROUND = 450.0f;       // kTurnPForRotatePGS
    static constexpr float K_TURN_P_AIR = 375.0f;          // kTurnPForFreeFallPGS
    static constexpr float TURN_TORQUE_MAX_Y = 1e5f;

    static constexpr float K_JUMP_P = 500.0f;
    static constexpr float JUMP_FORCE_FACTOR_GROUND = 0.52f;  // PGSJumpForceAdjustment/1000
    static constexpr float JUMP_FORCE_FACTOR_AIR = 0.1f;
    static constexpr float JUMP_DURATION = 0.5f;

    static constexpr float GRAVITY = 196.2f;
    static constexpr float WORLD_DT = 1.0f / 240.0f;

    static constexpr float STEEP_SLOPE_ANGLE = 0.01745f;  // cos(89°) 
    float jumpPower = 50.0f;
    float walkSpeed = 16.0f;
    float hipHeight = 0.0f;

    // State
    bool grounded = false;
    bool wasGrounded = false;
    bool jumping = false;
    float jumpTimer = 0.0f;
    float jumpCooldown = 0.0f;

    bool hipEnabled = false;
    float cachedTargetY = 0.0f;
    bool cachedNearFloor = false;
    glm::vec3 cachedDesiredVel{0.0f}; 
    float cachedControlScale = 1.0f;   
    float cachedDesiredAngVelY = 0.0f;  
    bool cachedHasMoveInput = false;
    float cachedFloorFriction = 0.0f;    
    float cachedIbodyY = 1.0f;         
    float cachedFloorDist = 9999.0f;         // distance from root to floor (for debug)
    bool hadFloorLastFrame = false;           // floor hysteresis
    float cachedFloorY = 0.0f;                // averaged floor Y (for debug)
    bool cachedFloorExists = false;        
    glm::vec3 cachedFloorNormal{0, 1, 0};
    glm::vec3 lastFloorNormal = glm::vec3(0, 1, 0);  
    float savedTorsoFriction = 0.3f;
    float savedHeadFriction = 0.3f;
    bool frictionZeroed = false;
    int ladderCheck = 0;
    bool facingLadder = false; 
    int jumpFrameCount = 0;
    bool prevFloorHit = false;
    glm::vec3 jumpDir{0, 1, 0};  // jump direction (modified when jumping from ladder)

    OWSolver::ConstraintBodyAngularVelocity* uprightConstraint = nullptr;
    OWSolver::Body* cachedFloorBodyPtr = nullptr;  // body hit by floor raycast
    glm::vec3 cachedCamLook{0, 0, -1};

    // debug raycat visualization (findFloor rays)
    struct DebugRay {
        glm::vec3 start;
        glm::vec3 end;
        glm::vec3 hitPoint;    // only valid if hit
        bool hit;
    };
    std::vector<DebugRay> debugRays;

    bool debugRaysEnabled = false;
    bool finished = false; 
    void substepCallback();
    State state = State::Running;
    float noFloorTimer = 0.0f;                // debounce period before Freefall (fallDelay = 0.125)
    float stateTimer = 0.0f;                  // for LANDED to RUNNING transition
    float landedTimer = 0.0f;
    bool findFloor(OWSolver::Body* root, float verticalVel, float legHeight, float hipHeight, float& outFloorY, glm::vec3& outFloorNormal);
    bool findCeiling(OWSolver::Body* root);
    bool findLadder(OWSolver::Body* root);
};