#include "OWPlayerController.hpp"
#include "OWWorld.hpp"
#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <set>

// helper conversion functions glm <-> raylib
static inline glm::vec3 toGlm(const Vector3& v) {
    return glm::vec3(v.x, v.y, v.z);
}

static inline Vector3 toRaylib(const glm::vec3& v) {
    return Vector3{v.x, v.y, v.z};
}

static float radWrap(float angle) {
    const float twoPi = 6.283185307179586476925286766559f;
    angle = std::fmod(angle, twoPi);
    if (angle > 3.14159265358979323846f) {
        angle -= twoPi;
    }
    if (angle < -3.14159265358979323846f) {
        angle += twoPi;
    }
    return angle;
}

static void collectAllDescendants(const OWSolver::Body* root, std::vector<const OWSolver::Body*>& out) {
    out.push_back(root);
    for (const OWSolver::Body* c : root->getChildren()) {
        collectAllDescendants(c, out);
    }
}

OWPlayerController::OWPlayerController(OWPart* p) : part(p) {
    if (OWWorld::Active && part->getBody()) {
        uprightConstraint = new OWSolver::ConstraintBodyAngularVelocity(part->getBody(), OWWorld::Active->getWorldBody());
        // X/Z: 4e5 (upright). Y: 0 (handled by direct torque in substepCallback)
        uprightConstraint->setMaxTorque(glm::vec3(4e5f, 0, 4e5f));
        uprightConstraint->setMinTorque(glm::vec3(-4e5f, 0, -4e5f));
        uprightConstraint->setUseIntegratedVelocities(false);
        OWWorld::Active->addPersistentConstraint(uprightConstraint);
        OWWorld::Active->setPreSubstepCallback([this]() { this->substepCallback(); });
    }
}

OWPlayerController::~OWPlayerController() {
    if (OWWorld::Active && uprightConstraint) {
        OWWorld::Active->removePersistentConstraint(uprightConstraint);
    }
    delete uprightConstraint;
}

float OWPlayerController::getFloorDist() const { return cachedFloorDist; }

void OWPlayerController::updateInput(float dt) {
    OWSolver::Body* body = part->getBody();
    if (!body || !OWWorld::Active) return;

    float camHeading = camera ? camera->getHeading() : 0.0f;
    bool characterFacesCamera = camera && camera->isCharacterFacingCamera();

    // find floor (HumanoidState::findFloor)
    float verticalVel = body->getLinearVelocity().y;
    float legHeight = 2.0f;
    float floorY = 0.0f;
    glm::vec3 floorNormal(0, 1, 0);
    bool floorHit = findFloor(body, verticalVel, legHeight, hipHeight, floorY, floorNormal);
    cachedFloorY = floorY;
    cachedFloorNormal = floorNormal;

    // target altitude/height
    float characterHipHeight = 0.5f * 2.0f + 2.0f + hipHeight;
    float rootY = body->getWorldPosition().y;
    float targetY = floorHit ? (floorY + characterHipHeight) : 0.0f;
    float floorDist = floorHit ? (rootY - floorY) : 9999.0f;
    cachedFloorDist = floorDist;

    // cachedNearFloor persists during coyote time
    if (floorHit) {
        cachedFloorExists = true;
    } else if (state == State::Freefall) {
        cachedFloorExists = false;
    }
    cachedNearFloor = cachedFloorExists;
    if (floorHit) {
        cachedTargetY = targetY;
    }

    // cache floor friction for movement force scaling
    if (floorHit && cachedFloorBodyPtr) {
        cachedFloorFriction = std::clamp(cachedFloorBodyPtr->getFriction(), 0.0f, 1.0f);
    } else {
        cachedFloorFriction = 0.0f;
    }

    float vy = body->getLinearVelocity().y;

    // findLadder
    // Running state checks every 2 frames (ladderCheckRate = 2)
    // Freefall state checks every frame (ladderCheckRate = 0)
    bool checkLadderThisFrame = false;
    if (state == State::Climbing) {
        checkLadderThisFrame = true;  // always check when climbing
    } else if (state == State::Freefall) {
        checkLadderThisFrame = true;  // every frame in freefall
    } else if (state == State::Running || state == State::Landed) {
        ladderCheck--;
        if (ladderCheck <= 0) {
            checkLadderThisFrame = true;
            ladderCheck = 2;  // ladderCheckRate for Running
        }
    }

    if (checkLadderThisFrame) {
        facingLadder = findLadder(body);
    }

    // humanoid state machine lool
    switch (state) {
        case State::Running:
            if (IsKeyDown(KEY_SPACE)) {
                state = State::Jumping;
                jumpTimer = JUMP_DURATION;
                noFloorTimer = 0.0f;
                jumpDir = glm::vec3(0, 1, 0);
                finished = false;  // reset
            }
            // FACE_LDR -> CLIMBING
            if (facingLadder) {
                state = State::Climbing;
                break;
            }
            if (!floorHit) {
                noFloorTimer += dt;
                if (noFloorTimer > 0.125f) {
                    state = State::Freefall;
                }
            } else {
                noFloorTimer = 0.0f;
            }
            break;

        case State::Jumping:
            jumpTimer -= dt;
            // state transition happens at 60Hz (doSimulatorStateTable)
            // this gives the character 1 extra frame to move away from trusses before findLadder is called in Freefall state.
            if (finished || jumpTimer <= 0.0f) {
                state = State::Freefall;
                finished = false;
                jumpDir = glm::vec3(0, 1, 0);  // reset jump dir
            }
            break;

        case State::Freefall:
            // FACE_LDR -> CLIMBING
            if (facingLadder) {
                state = State::Climbing;
                break;
            }
            if (floorHit && vy <= 0.0f) {
                state = State::Landed;
                landedTimer = 0.05f;
            }
            break;

        case State::Landed:
            landedTimer -= dt;
            if (landedTimer <= 0.0f) {
                state = State::Running;
            }
            if (!floorHit) {
                state = State::Freefall;
            }
            break;

        case State::Climbing:
            if (!facingLadder) {
                state = State::Running;
                break;
            }
            if (IsKeyDown(KEY_SPACE)) {
                state = State::Jumping;
                jumpTimer = JUMP_DURATION;

                glm::vec3 lookDir = -glm::vec3(body->getWorldOrientation()[2]);
                lookDir.y = 0;
                lookDir = glm::normalize(lookDir);
                jumpDir = glm::vec3(0, 1, 0) - lookDir;
                jumpDir = glm::normalize(jumpDir);
                
                facingLadder = false;
                cachedFloorExists = false;
                cachedNearFloor = true;
                finished = false;
                break;
            }
            break;
    }

    OWSolver::Body* torso = body;
    OWSolver::Body* head = nullptr;
    if (!body->getChildren().empty()) {
        torso = body->getChildren()[0];  // Torso is the only child of Root
        for (OWSolver::Body* c : torso->getChildren()) {
            if (c->getName() == "Head") {
                head = c; 
                
                break;
            }
        }
    }

    if (OWWorld::Active) {
        if (state == State::Jumping) {
            OWWorld::Active->setBodyContactResponse(torso, false);
            OWWorld::Active->updateBodyFriction(torso, 0.0f);
            if (head) OWWorld::Active->updateBodyFriction(head, 0.0f);
        } else if (state == State::Freefall) {
            OWWorld::Active->setBodyContactResponse(torso, true);
            OWWorld::Active->updateBodyFriction(torso, 0.0f);
            if (head) OWWorld::Active->updateBodyFriction(head, 0.0f);
        } else {
            OWWorld::Active->setBodyContactResponse(torso, true);
            OWWorld::Active->updateBodyFriction(torso, 0.3f);
            if (head) OWWorld::Active->updateBodyFriction(head, 0.3f);
        }
    }

    // -- Derived flags --
    // grounded requires state AND floor hit (to avoid stuff like "grounded" flash when transitioning from Climbing to Running in mid-air)
    grounded = (state == State::Running || state == State::Landed) && floorHit;
    jumping = (state == State::Jumping);
    hipEnabled = (state == State::Running || state == State::Landed) && floorHit;

    // Movement!!!
    glm::vec3 forward(-std::sin(camHeading), 0, -std::cos(camHeading));
    glm::vec3 right(std::cos(camHeading), 0, -std::sin(camHeading));

    glm::vec3 inputDir(0.0f);
    if (IsKeyDown(KEY_W)) {
        inputDir += forward;
    }

    if (IsKeyDown(KEY_S)) {
        inputDir -= forward;
    }

    if (IsKeyDown(KEY_D)) {
        inputDir += right;
    }

    if (IsKeyDown(KEY_A)) {
        inputDir -= right;
    }

    cachedHasMoveInput = ( glm::length(inputDir) > 0.001f );

    cachedControlScale = grounded ? 1.0f : 0.1f;
    if (cachedHasMoveInput) {
        inputDir = glm::normalize(inputDir);
        cachedDesiredVel = inputDir * walkSpeed;
    } else {
        cachedDesiredVel = glm::vec3(0.0f);
    }
    cachedDesiredVel.y = 0;

    // CLIMBING VELOCITY OVERRIDE (RunningBase::onSimulatorStepImpl)
    // when facing ladder, override desired velocity to be vertical only
    if (state == State::Climbing && facingLadder) {
        glm::vec3 forwardDir = -glm::vec3(body->getWorldOrientation()[2]);
        forwardDir.y = 0;
        forwardDir = glm::normalize(forwardDir);
        
        float moveDir = 1.0f;
        if (cachedHasMoveInput) {
            float dot = glm::dot(forwardDir, glm::normalize(inputDir));
            if (dot < -0.2f) {
                moveDir = -1.0f;
            }
        }

        // can only climb down if not on floor
        if (moveDir > 0.0f || !floorHit) {
            float speed = glm::length(cachedDesiredVel);
            if (speed < 0.1f) {
                cachedDesiredVel.y = 0.01f * moveDir;  // slow drift up
            } else {
                cachedDesiredVel.y = 0.7f * speed * moveDir;  // 0.7 * walkSpeed
            }
            cachedDesiredVel.x = 0;
            cachedDesiredVel.z = 0;
        }
    }

    // iif floor normal is too steep (e.g. hit the side of a platform edge), push the character downhill to make them fall off.
    if (floorHit && state == State::Running) {
        float dot = cachedFloorNormal.y;  // normal dot (0,1,0) = normal.y
        if (dot < STEEP_SLOPE_ANGLE) {
            glm::vec3 downhill = -(glm::vec3(0, 1, 0) - (dot * cachedFloorNormal));
            float downhillLen = glm::length(downhill);
            if (downhillLen > 0.0001f) {
                downhill /= downhillLen;  // normalize
                glm::vec3 inputOntoSurface = cachedDesiredVel - (glm::dot(cachedDesiredVel, cachedFloorNormal) * cachedFloorNormal);
                cachedDesiredVel = inputOntoSurface + (downhill * walkSpeed) - (glm::dot(inputOntoSurface, downhill) * downhill);
            }
        }
    }

    // character Y rotation (calcDesiredWalkVelocity)
    static constexpr float AUTO_TURN_SPEED = 8.0f;
    static constexpr float FREEFALL_TURN_DEADZONE = 0.2f;
    static constexpr float FREEFALL_TURN_SPEED = 8.0f;

    if (characterFacesCamera) {
        // INSTANT ROTATION (matches Roblox setFirstPersonRotationalVelocity)
        // Done at 60Hz (updateInput) to match Roblox's onCameraHeartbeat timing.
        // This allows Bullet's broadphase to update, enabling corner clipping.
        glm::vec3 newForward = glm::normalize(camera->getLookHorizontal());
        glm::vec3 worldUp(0, 1, 0);
        glm::vec3 newRight = glm::normalize(glm::cross(newForward, worldUp));
        glm::vec3 newUp = glm::cross(newRight, newForward);

        glm::mat3 newR(newRight, newUp, -newForward);
        OWSolver::CoordinateFrame cf(newR, body->getWorldPosition());
        body->setWorldCFrame(cf);
        body->setAngularVelocity(glm::vec3(0));
        
        cachedDesiredAngVelY = 0.0f;
    } else if (cachedHasMoveInput) {
        glm::mat3 R = body->getWorldOrientation();
        glm::vec3 charForward = -glm::vec3(R[2]);
        float currentHeading = std::atan2(charForward.x, charForward.z);
        float desiredHeading = std::atan2(inputDir.x, inputDir.z);
        float deltaAngle = radWrap(desiredHeading - currentHeading);

        if (grounded) {
            cachedDesiredAngVelY = AUTO_TURN_SPEED * deltaAngle;
        } else {
            if (std::abs(deltaAngle) > FREEFALL_TURN_DEADZONE) {
                cachedDesiredAngVelY = FREEFALL_TURN_SPEED * (deltaAngle > 0 ? 1.0f : -1.0f);
            } else {
                cachedDesiredAngVelY = 0.25f * FREEFALL_TURN_SPEED * (deltaAngle > 0 ? 1.0f : -1.0f) * std::abs(deltaAngle);
            }
        }
    } else {
        cachedDesiredAngVelY = 0.0f;
    }

    // -------- Cache body-space Y inertia --------
    {
        glm::vec3 IbodyDiag = body->getBranchIBodyV3();
        cachedIbodyY = IbodyDiag.y;
        if (!std::isfinite(cachedIbodyY) || cachedIbodyY < 0.001f) cachedIbodyY = 1.0f;
    }

    if (state == State::Jumping) {
        uprightConstraint->setTarget(glm::vec3(0));
    } else {
        glm::mat3 R = body->getWorldOrientation();
        glm::vec3 yAxisWorld = R * glm::vec3(0, 1, 0);
        glm::vec3 tiltWorld = glm::cross(glm::vec3(0, 1, 0), yAxisWorld);
        glm::vec3 tiltRoot = glm::transpose(R) * tiltWorld;
        glm::vec3 angVelWorld = body->getAngularVelocity();
        glm::vec3 angVelRoot = glm::transpose(R) * angVelWorld;

        float kP = grounded ? (state == State::Landed ? 7500.0f : K_BALANCE_P_GROUND) : K_BALANCE_P_AIR;
        glm::vec3 desiredAccelRoot = -kP * tiltRoot - K_BALANCE_D * angVelRoot;
        glm::vec3 targetAngVelRoot = angVelRoot + desiredAccelRoot * WORLD_DT;
        targetAngVelRoot.y = angVelRoot.y;
        uprightConstraint->setTarget(targetAngVelRoot);
    }
}

void OWPlayerController::substepCallback() {
    OWSolver::Body* body = part->getBody();
    if (!body) return;

    float mass = body->getBranchMass();
    if (!std::isfinite(mass) || mass < 0.001f) return;

    glm::vec3 pos = body->getWorldPosition();
    glm::vec3 vel = body->getLinearVelocity();
    if (!std::isfinite(pos.x) || !std::isfinite(vel.x)) {
        body->setLinearVelocity(glm::vec3(0.0f));
        body->setAngularVelocity(glm::vec3(0.0f));
        return;
    }

    // shift lock / first person
    // 240Hz instead of using the 60Hz cached value.
    // this makes rotation perfectly smooth at any fps
    bool characterFacesCamera = camera && camera->isCharacterFacingCamera();
    if (characterFacesCamera) {
        glm::vec3 newForward = glm::normalize(camera->getLookHorizontal());
        glm::vec3 worldUp(0, 1, 0);
        glm::vec3 newRight = glm::normalize(glm::cross(newForward, worldUp));
        glm::vec3 newUp = glm::cross(newRight, newForward);

        glm::mat3 newR(newRight, newUp, -newForward);
        OWSolver::CoordinateFrame cf(newR, pos);
        body->setWorldCFrame(cf);
        body->setAngularVelocity(glm::vec3(0));
    }

    // JUMPING
    if (state == State::Jumping) {
        if (findCeiling(body)) {
            finished = true;
            return;
        }

        // Jumping.cpp jumpVelocity = velocity dot jumpDir
        float jumpVel = glm::dot(vel, jumpDir);
        float yAccelDesired = K_JUMP_P * (jumpPower - jumpVel);
        if (yAccelDesired <= 0.0f) {
            finished = true; 
            // Do NOT change state here btw state machine handles it at 60Hz
        } else {
            float factor = cachedNearFloor ? JUMP_FORCE_FACTOR_GROUND : JUMP_FORCE_FACTOR_AIR;
            body->accumulateForce(jumpDir * (mass * yAccelDesired * factor));
        }
        return;
    }

    // CLIMBING SLOP
    if (state == State::Climbing) {
        // movement force
        glm::vec3 currentVel = body->getLinearVelocity();
        glm::vec3 velError = cachedDesiredVel - currentVel;
        
        glm::vec3 desiredAccel = K_MOVE_P_PGS * velError;

        glm::vec3 horizontalAccel(desiredAccel.x, 0.0f, desiredAccel.z);
        float maxAccel = 500.0f;
        float accelMag = glm::length(horizontalAccel);
        if (accelMag > maxAccel) {
            horizontalAccel = horizontalAccel * (maxAccel / accelMag);
            desiredAccel.x = horizontalAccel.x;
            desiredAccel.z = horizontalAccel.z;
        }
        
        glm::vec3 deltaForce = mass * desiredAccel;
        
        deltaForce.y += mass * GRAVITY; 

        if (cachedFloorFriction > 0.0f) {
            deltaForce.x *= cachedFloorFriction;
            deltaForce.z *= cachedFloorFriction;
        }
        
        body->accumulateForce(deltaForce);

        {
            float currentAngVelY = body->getAngularVelocity().y;
            float desiredTorqueY = K_TURN_P_GROUND * cachedIbodyY * (0.0f - currentAngVelY);
            desiredTorqueY = std::clamp(desiredTorqueY, -TURN_TORQUE_MAX_Y, TURN_TORQUE_MAX_Y);
            body->accumulateTorque(glm::vec3(0, desiredTorqueY, 0));
        }
        return;
    }

    if (state == State::Freefall) {
        glm::vec3 currentVel = body->getLinearVelocity();
        glm::vec3 velError = cachedDesiredVel - currentVel;
        velError.y = 0;

        glm::vec3 desiredAccel = K_MOVE_P_PGS * velError;
        glm::vec3 horizontalAccel(desiredAccel.x, 0.0f, desiredAccel.z);

        constexpr float MAX_AIR_MOVE_ACCEL = 143.0f;
        float accelMag = glm::length(horizontalAccel);
        if (accelMag > MAX_AIR_MOVE_ACCEL) {
            horizontalAccel = horizontalAccel * (MAX_AIR_MOVE_ACCEL / accelMag);
        }
        body->accumulateForce(mass * horizontalAccel);

        // y rotation torque
        float currentAngVelY = body->getAngularVelocity().y;
        float desiredTorqueY = K_TURN_P_AIR * cachedIbodyY * (cachedDesiredAngVelY - currentAngVelY);
        desiredTorqueY -= body->getExternalTorque().y;
        float freefallTorqueMax = cachedIbodyY * 120000.0f;
        desiredTorqueY = std::clamp(desiredTorqueY, -freefallTorqueMax, freefallTorqueMax);
        body->accumulateTorque(glm::vec3(0, desiredTorqueY, 0));
        return;
    }

    // RUNNING
    // (PGSFixGroundSinking = true)
    if (hipEnabled) {
        float error = cachedTargetY - pos.y;
        float yAccelDesired = K_ALTITUDE_P * error - K_ALTITUDE_D * vel.y;

        if (yAccelDesired > 0.0f) {
            float currentForceY = body->getExternalForce().y;
            float currentAccelY = currentForceY / mass;
            if (yAccelDesired > currentAccelY) {
                float scaleFactor = 1.0f;
                float accelerationDelta = (yAccelDesired * ALTITUDE_SCALE) - currentAccelY;
                float deltaForce = mass * accelerationDelta;
                deltaForce = std::clamp(deltaForce, -1e7f, 1e7f);
                body->accumulateForce(glm::vec3(0, deltaForce * scaleFactor, 0));
            }
        }
    }


    // more movement force stuff
    // i forgot why i override y here
    {
        glm::vec3 currentVel = body->getLinearVelocity();
        glm::vec3 velError = cachedDesiredVel - currentVel;
        velError.y = 0;
        glm::vec3 desiredAccel = K_MOVE_P_PGS * velError;
        glm::vec3 currentAccel = body->getExternalForce() / mass;
        currentAccel.y = 0;  
        glm::vec3 deltaAccel = desiredAccel - currentAccel;
        deltaAccel.y = 0;

        // clamp horizontal
        glm::vec3 horizontalAccel(deltaAccel.x, 0.0f, deltaAccel.z);
        float maxAccel = cachedFloorExists ? 500.0f : 143.0f;
        float accelMag = glm::length(horizontalAccel);
        if (accelMag > maxAccel) {
            horizontalAccel = horizontalAccel * (maxAccel / accelMag);
            deltaAccel.x = horizontalAccel.x;
            deltaAccel.z = horizontalAccel.z;
        }

        body->accumulateForce(mass * deltaAccel);
    }

    {
        float currentAngVelY = body->getAngularVelocity().y;
        float desiredTorqueY = K_TURN_P_GROUND * cachedIbodyY * (cachedDesiredAngVelY - currentAngVelY);
        desiredTorqueY = std::clamp(desiredTorqueY, -TURN_TORQUE_MAX_Y, TURN_TORQUE_MAX_Y);
        body->accumulateTorque(glm::vec3(0, desiredTorqueY, 0));
    }
}

// HumanoidState::findFloor !!!!!!!!!!!!!!!!! 
bool OWPlayerController::findFloor(OWSolver::Body* root, float verticalVel, float legHeight, float hipHeight, float& outFloorY, glm::vec3& outFloorNormal) {
    if (!OWWorld::Active) return false;

    debugRays.clear();
    cachedFloorBodyPtr = nullptr;
    OWSolver::CoordinateFrame torsoCF = root->getWorldCFrame();
    glm::vec3 torsoPos = torsoCF.translation;
    glm::mat3 torsoRot = torsoCF.rotation;

    glm::vec3 halfSize(0.8f, 0.8f, 0.4f);

    float maxDistance = 1.0f;
    float hysteresis = hadFloorLastFrame ? 1.5f : 1.1f;
    if (std::abs(verticalVel) > 100.0f) {
        hysteresis += std::abs(verticalVel) / 100.0f;
    }
    maxDistance += hysteresis * legHeight;
    maxDistance += hysteresis * hipHeight;

    std::vector<const OWSolver::Body*> ignore;
    collectAllDescendants(root, ignore);

    float zOffsets[] = {0.0f, 1.0f, -1.0f, 2.0f, -2.0f}; // raycast offsets
    glm::vec3 hitAccumulator(0.0f);
    int hitCount = 0;
    bool found = false;

    for (int z = 0; z < 5; z++) {
        glm::vec3 localOffset(0.0f, -halfSize.y, zOffsets[z] * halfSize.z);
        glm::vec3 worldOrigin = torsoPos + torsoRot * localOffset;
        glm::vec3 rayEnd = worldOrigin - glm::vec3(0, maxDistance, 0);

        OWWorld::RayHit hit;
        bool didHit = OWWorld::Active->raycastClosest(worldOrigin, rayEnd, hit, ignore);

        DebugRay dr;
        dr.start = worldOrigin;
        dr.end = rayEnd;
        dr.hit = didHit;
        dr.hitPoint = didHit ? hit.point : glm::vec3(0);
        debugRays.push_back(dr);

        if (didHit) {
            hitAccumulator += hit.point;
            hitCount++;
            bool isCenterRay = (zOffsets[z] < 2.0f && zOffsets[z] > -2.0f);
            if (isCenterRay && !found) {
                outFloorNormal = hit.normal;
                cachedFloorBodyPtr = hit.body;
                found = true;
            }
        }
    }

    if (!found) {
        for (int x = -1; x <= 1; x += 2) {
            for (int zz = -1; zz <= 1; zz += 2) {
                glm::vec3 localOffset(x * halfSize.x, -halfSize.y, zz * halfSize.z);
                glm::vec3 worldOrigin = torsoPos + torsoRot * localOffset;
                glm::vec3 rayEnd = worldOrigin - glm::vec3(0, maxDistance, 0);

                OWWorld::RayHit hit;
                bool didHit = OWWorld::Active->raycastClosest(worldOrigin, rayEnd, hit, ignore);

                DebugRay dr;
                dr.start = worldOrigin;
                dr.end = rayEnd;
                dr.hit = didHit;
                dr.hitPoint = didHit ? hit.point : glm::vec3(0);
                debugRays.push_back(dr);

                if (didHit) {
                    hitAccumulator += hit.point;
                    hitCount++;
                    if (!found) {
                        outFloorNormal = hit.normal;
                        found = true;
                    }
                }
            }
        }
    }

    if (found && hitCount > 0) {
        outFloorY = hitAccumulator.y / hitCount;
        hadFloorLastFrame = true;
        return true;
    } else {
        hadFloorLastFrame = false;
        return false;
    }
}

// Jumping::findCeiling!!!!!!!!!!!!!! 
bool OWPlayerController::findCeiling(OWSolver::Body* root) {
    if (!OWWorld::Active) return false;

    OWSolver::CoordinateFrame torsoCF = root->getWorldCFrame();
    glm::vec3 torsoPos = torsoCF.translation;
    glm::mat3 torsoRot = torsoCF.rotation;

    glm::vec3 halfSize(0.8f, 0.8f, 0.4f);
    float maxDistance = 1.0f + 2.0f * 1.5f;

    std::vector<const OWSolver::Body*> ignore;
    collectAllDescendants(root, ignore);

    {
        glm::vec3 localOffset(0.0f, -halfSize.y, 0.0f);
        glm::vec3 worldOrigin = torsoPos + torsoRot * localOffset;
        glm::vec3 rayEnd = worldOrigin + glm::vec3(0, maxDistance, 0);

        OWWorld::RayHit hit;
        if (OWWorld::Active->raycastClosest(worldOrigin, rayEnd, hit, ignore)) {
            return true;
        }
    }

    for (int x = -1; x <= 1; x += 2) {
        for (int z = -1; z <= 1; z += 2) {
            glm::vec3 localOffset(x * halfSize.x, -halfSize.y, z * halfSize.z);
            glm::vec3 worldOrigin = torsoPos + torsoRot * localOffset;
            glm::vec3 rayEnd = worldOrigin + glm::vec3(0, maxDistance, 0);

            OWWorld::RayHit hit;
            if (OWWorld::Active->raycastClosest(worldOrigin, rayEnd, hit, ignore)) {
                return true;
            }
        }
    }

    return false;
}

void OWPlayerController::renderDebugRays() {
    if (!debugRaysEnabled) return;

    for (const auto& r : debugRays) {
        Vector3 start = toRaylib(r.start);
        if (r.hit) {
            Vector3 hitP = toRaylib(r.hitPoint);
            DrawLine3D(start, hitP, GREEN);
            DrawSphere(hitP, 0.08f, GREEN);
            Vector3 end = toRaylib(r.end);
            DrawLine3D(hitP, end, (Color){100, 40, 40, 255});
        } else {
            Vector3 end = toRaylib(r.end);
            DrawLine3D(start, end, RED);
        }
    }

    bool anyHit = false;
    for (const auto& r : debugRays) {
        if (r.hit) { anyHit = true; break; }
    }
    if (anyHit) {
        OWSolver::Body* body = part->getBody();
        if (body) {
            glm::vec3 pos = body->getWorldPosition();
            glm::vec3 avgPoint(pos.x, cachedFloorY, pos.z);
            DrawSphere(toRaylib(avgPoint), 0.15f, YELLOW);
        }
    }
}

// HumanoidState::findLadder and findPrimitiveInLadderZone!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
bool OWPlayerController::findLadder(OWSolver::Body* root) {
    if (!OWWorld::Active) return false;

    OWSolver::CoordinateFrame torsoCF = root->getWorldCFrame();
    glm::vec3 torsoPos = torsoCF.translation;
    glm::mat3 torsoRot = torsoCF.rotation;
    glm::vec3 torsoLook = -glm::vec3(torsoRot[2]);  // lookVector = -Z

    // constants (EnableHipHeight=false, FixSlowLadderClimb=false)
    constexpr float lowLadderSearch = 2.7f;
    constexpr float searchDepth = 0.7f;          // (R6)
    constexpr float ladderSearchDistance = 1.05f;      /* 0.7 * 1.5 */
    constexpr float maxClimbDistance = 2.45f;          // HumanoidState.h maxClimbDistance()
    constexpr float sampleSpacingVal = 1.0f / 7.0f;
    constexpr float heightScale = 1.0f;               // getCharacterHipHeight() / 3.0 = 3.0/3.0
    constexpr float spacing = sampleSpacingVal * heightScale;  // 0.142857

    // build ignore list: root + all children
    std::vector<const OWSolver::Body*> ignore;
    ignore.push_back(root);
    for (const OWSolver::Body* c : root->getChildren()) {
        ignore.push_back(c);
    }

    // step 1: AABB Check (findPrimitiveInLadderZone)
    float bottom = -lowLadderSearch;  // -2.7
    float top = 0.0f;
    float radius = 0.5f * ladderSearchDistance;  // 0.525
    glm::vec3 center = torsoPos + (torsoLook * ladderSearchDistance * 0.5f);
    glm::vec3 aabbMin = center + glm::vec3(-radius, bottom, -radius);
    glm::vec3 aabbMax = center + glm::vec3(radius, top, radius);

    // get bodies touching the AABB
    std::vector<OWSolver::Body*> bodiesInZone = OWWorld::Active->getBodiesInAABB(aabbMin, aabbMax, ignore);
    if (bodiesInZone.empty()) {
        return false;  // no parts in ladder zone so no ladder was found
    }

    // for later stuff like filtering during raycasts
    std::set<uint64_t> visibleUids;
    for (auto* b : bodiesInZone) {
        visibleUids.insert(b->getUID());
    }

    // step b the 27 motherfucking scary raycasts
    int topRay = static_cast<int>(lowLadderSearch / spacing) + 10;  // 18 + 10 = 28

    float first_space = 0.0f;    // needs to be < 2.45f
    float first_step = 0.0f;     // needs to be > 0 and < 2.45f
    float second_step = -1.0f;

    bool look_for_space = true;
    bool look_for_step = false;

    for (int i = 1; i < topRay; i++) {
        float distanceFromBottom = (float)i * spacing;
        // origin: (0, -2.7 + distanceFromBottom, 0) relative to torso
        glm::vec3 originOnTorso(0.0f, -lowLadderSearch + distanceFromBottom, 0.0f);
        glm::vec3 rayOrigin = torsoPos + torsoRot * originOnTorso;
        glm::vec3 rayEnd = rayOrigin + torsoLook * ladderSearchDistance;

        // raycast
        OWWorld::RayHit hit;
        bool didHit = OWWorld::Active->raycastClosest(rayOrigin, rayEnd, hit, ignore);

        // only count hits on bodies in the ladder zone
        if (didHit && hit.body && visibleUids.count(hit.body->getUID()) == 0) {
            didHit = false;  // hit a body not in the box, treat as miss
        }

        // truss check, if a ray hits a truss, immediately climbable
        if (didHit && hit.body && hit.body->getIsTruss()) {
            return true;
        }

        float mag = didHit ? glm::distance(rayOrigin, hit.point) : 9999.0f;

        if (didHit && mag < searchDepth) {
            if (look_for_space) {
                first_space = distanceFromBottom;
                look_for_space = false;
                look_for_step = true;
            } else if (!look_for_step && second_step < 0.0f) {
                // found a second step after the gap
                second_step = distanceFromBottom;
            }
        } else {
            // this is a gap
            if (look_for_step) {
                // Found the end of the step gap begins
                first_step = distanceFromBottom - first_space;
                look_for_step = false;
            }
        }
    }

    // step 3 : climb conditions
    if (first_space < maxClimbDistance * heightScale &&
        first_step > 0.0f && 
        first_step < maxClimbDistance * heightScale &&
        (second_step >= 0.0f || first_space > (maxClimbDistance * heightScale / 5.0f))
    ) {
        return true;
    }

    return false;
}