#include "libowsolver/ConstraintTypes.hpp"
#include "libowsolver/Body.hpp"
//#include <glm/gtx/norm.hpp>
#include <algorithm>
#include <cmath>

namespace OWSolver {
    static void generateOrthonormalBasis( glm::vec3& t1, glm::vec3& t2, const glm::vec3& n) {
        if (std::abs(n.x) < 0.9f) {
            t1 = glm::normalize(glm::cross(n, glm::vec3(1, 0, 0)));
        } else {
            t1 = glm::normalize(glm::cross(n, glm::vec3(0, 1, 0)));
        }
        t2 = glm::cross(t1, n);
    }

    //
    // ConstraintBallInSocket
    //
    void ConstraintBallInSocket::buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) {
        // transform to world space
        glm::vec3 worldPointA = bodyA.orientation * pointA + bodyA.position;
        glm::vec3 worldPointB = bodyB.orientation * pointB + bodyB.position;

        glm::vec3 relA = worldPointA - bodyA.position;
        glm::vec3 relB = worldPointB - bodyB.position;

        // build 3 Jacobian rows (X, Y, Z)
        jacobian[0].a.lin = glm::vec3( 1, 0, 0);
        jacobian[0].b.lin = glm::vec3(-1, 0, 0);
        jacobian[0].a.ang = -glm::vec3(0, -relA.z, relA.y);
        jacobian[0].b.ang = glm::vec3(0, -relB.z, relB.y);

        jacobian[1].a.lin = glm::vec3( 0, 1, 0);
        jacobian[1].b.lin = glm::vec3( 0,-1, 0);
        jacobian[1].a.ang = -glm::vec3(relA.z, 0, -relA.x);
        jacobian[1].b.ang = glm::vec3(relB.z, 0, -relB.x);

        jacobian[2].a.lin = glm::vec3( 0, 0, 1);
        jacobian[2].b.lin = glm::vec3(0, 0,-1);
        jacobian[2].a.ang = -glm::vec3(-relA.y, relA.x, 0);
        jacobian[2].b.ang = glm::vec3(-relB.y, relB.x, 0);

        // velocity constraint: relative velocity at pivot points should be zero
        glm::vec3 vA = bodyA.integratedLinearVelocity + glm::cross(bodyA.integratedAngularVelocity, relA);
        glm::vec3 vB = bodyB.integratedLinearVelocity + glm::cross(bodyB.integratedAngularVelocity, relB);
        glm::vec3 deltaV = vB - vA;

        // position constraint: stabilization,
        glm::vec3 deltaP = worldPointB - worldPointA;
        float errorLength = glm::length(deltaP);
        float correctionLength = std::min(config.ballInSocketCorrectionDamping * errorLength, config.ballInSocketMaxCorrectionByStabilization);

        glm::vec3 stabilization(0);
        if (errorLength > 0.0f) {
            stabilization = deltaP * (correctionLength / errorLength);
        }

        ConstraintVariables::setReaction(velStage, deltaV);
        ConstraintVariables::setReaction(posStage, stabilization);
    }

    ConstraintBallInSocket::Convergence ConstraintBallInSocket::testPGSConvergence( const float* disp,const float* residuals, const float* deltaResiduals, const SolverConfig& config) {
        float t = residuals[0]*residuals[0] + residuals[1]*residuals[1] + residuals[2]*residuals[2];
        float threshold = config.inconsistentConstraintBallInSocketResidualThreshold;

        if (t < threshold * threshold) {
            float s = deltaResiduals[0]*deltaResiduals[0] + deltaResiduals[1]*deltaResiduals[1] + deltaResiduals[2]*deltaResiduals[2];
            float dt = config.inconsistentConstraintDeltaThreshold;
            if (s < dt * dt * threshold * threshold) {
                return Convergence_Converges;
            }
        } else {
            float s = deltaResiduals[0]*deltaResiduals[0] + deltaResiduals[1]*deltaResiduals[1] + deltaResiduals[2]*deltaResiduals[2];
            if (s < config.inconsistentConstraintDeltaThreshold * config.inconsistentConstraintDeltaThreshold * t) {
                return Convergence_Diverges;
            }
        }
        return Convergence_Undetermined;
    }

    //
    // ConstraintAlign2Axes
    //
    void ConstraintAlign2Axes::setAxisA(const glm::vec3& a) {
        axisA = a;
    }

    void ConstraintAlign2Axes::setAxisB(const glm::vec3& b) {
        axisB = b;
        generateOrthonormalBasis(orthogonalAxisB1, orthogonalAxisB2, axisB);
    }

    void ConstraintAlign2Axes::buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) {
        glm::mat3 rA = getBodyA()->getWorldCFrame().rotation;
        glm::mat3 rB = getBodyB() ? getBodyB()->getWorldCFrame().rotation : glm::mat3(1.0f);

        glm::vec3 oldOrthoB1 = worldSpaceOrthogonalB1;
        glm::vec3 oldOrthoB2 = worldSpaceOrthogonalB2;

        worldSpaceOrthogonalB1 = rB * orthogonalAxisB1;
        worldSpaceOrthogonalB2 = rB * orthogonalAxisB2;
        glm::vec3 worldAxisB = rB * axisB;
        glm::vec3 worldAxisA = rA * axisA;

        jacobian[0].a.lin = glm::vec3(0);
        jacobian[0].b.lin = glm::vec3(0);
        jacobian[0].a.ang = worldSpaceOrthogonalB1;
        jacobian[0].b.ang = -worldSpaceOrthogonalB1;

        jacobian[1].a.lin = glm::vec3(0);
        jacobian[1].b.lin = glm::vec3(0);
        jacobian[1].a.ang = worldSpaceOrthogonalB2;
        jacobian[1].b.ang = -worldSpaceOrthogonalB2;

        // Velocity stage
        glm::vec3 relAngVel = bodyB.integratedAngularVelocity - bodyA.integratedAngularVelocity;
        glm::vec3 worldVelImpulse = velStage[0].impulse * oldOrthoB1 + velStage[1].impulse * oldOrthoB2;

        velStage[0].impulse = glm::dot(worldSpaceOrthogonalB1, worldVelImpulse);
        velStage[0].reaction = glm::dot(worldSpaceOrthogonalB1, relAngVel);
        velStage[1].impulse = glm::dot(worldSpaceOrthogonalB2, worldVelImpulse);
        velStage[1].reaction = glm::dot(worldSpaceOrthogonalB2, relAngVel);

        // position stage
        glm::vec3 angularError = glm::cross(worldAxisA, worldAxisB);
        float cosAngle = glm::dot(worldAxisA, worldAxisB);
        float maxCorrectiveAngle = config.align2AxesMaxCorrectiveAngle * (3.14159265f / 180.0f);

        if (cosAngle < std::cos(maxCorrectiveAngle)) {
            float sinAngle = glm::length(angularError);
            float scale = (sinAngle > 0.00001f) ? (std::sin(maxCorrectiveAngle) / sinAngle) : 0.0f;
            angularError *= scale;
        }
        angularError *= config.align2AxesCorrectionDamping;

        glm::vec3 worldPosImpulse = posStage[0].impulse * oldOrthoB1 + posStage[1].impulse * oldOrthoB2;
        posStage[0].impulse = glm::dot(worldSpaceOrthogonalB1, worldPosImpulse);
        posStage[0].reaction = glm::dot(worldSpaceOrthogonalB1, angularError);
        posStage[1].impulse = glm::dot(worldSpaceOrthogonalB2, worldPosImpulse);
        posStage[1].reaction = glm::dot(worldSpaceOrthogonalB2, angularError);
    }

    ConstraintAlign2Axes::Convergence ConstraintAlign2Axes::testPGSConvergence( const float* disp, const float* residuals, const float* deltaResiduals, const SolverConfig& config) {
        float t = residuals[0]*residuals[0] + residuals[1]*residuals[1];
        float threshold = config.inconsistentConstraintAlign2AxesThreshold;
        if (t < threshold * threshold) {
            float s = deltaResiduals[0]*deltaResiduals[0] + deltaResiduals[1]*deltaResiduals[1];
            if (s < config.inconsistentConstraintDeltaThreshold * config.inconsistentConstraintDeltaThreshold * threshold * threshold) return Convergence_Converges;
        } else {
            float s = deltaResiduals[0]*deltaResiduals[0] + deltaResiduals[1]*deltaResiduals[1];
            if (s < config.inconsistentConstraintDeltaThreshold * config.inconsistentConstraintDeltaThreshold * t) return Convergence_Diverges;
        }
        return Convergence_Undetermined;
    }

    //
    // ConstraintLinearVelocity
    //
    void ConstraintLinearVelocity::buildEquation( ConstraintJacobianPair* jacobian, uint8_t* useBlock,ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) {
        useBlock[0] = false;
        useBlock[1] = false;
        useBlock[2] = false;

        jacobian[0].a.lin = glm::vec3( 1, 0, 0);
        jacobian[0].b.lin = glm::vec3(-1, 0, 0);
        jacobian[0].a.ang = glm::vec3(0);
        jacobian[0].b.ang = glm::vec3(0);

        jacobian[1].a.lin = glm::vec3( 0, 1, 0);
        jacobian[1].b.lin = glm::vec3( 0,-1, 0);
        jacobian[1].a.ang = glm::vec3(0);
        jacobian[1].b.ang = glm::vec3(0);

        jacobian[2].a.lin = glm::vec3( 0, 0, 1);
        jacobian[2].b.lin = glm::vec3( 0, 0,-1);
        jacobian[2].a.ang = glm::vec3(0);
        jacobian[2].b.ang = glm::vec3(0);

        glm::vec3 relVel = bodyB.integratedLinearVelocity - bodyA.integratedLinearVelocity;

        ConstraintVariables::setReaction(velStage, desiredVelocity + relVel);
        ConstraintVariables::setMinImpulses(velStage, -maxForce * dt);
        ConstraintVariables::setMaxImpulses(velStage, maxForce * dt);

        ConstraintVariables::setReaction(posStage, glm::vec3(0));
        ConstraintVariables::setMinImpulses(posStage, glm::vec3(0));
        ConstraintVariables::setMaxImpulses(posStage, glm::vec3(0));
    }

    //
    // ConstraintBodyAngularVelocity
    //
    void ConstraintBodyAngularVelocity::buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) {
        useBlock[0] = false;
        useBlock[1] = false;
        useBlock[2] = false;

        // Axes in world space from bodyA orientation
        jacobian[0].a.lin = glm::vec3(0);
        jacobian[0].b.lin = glm::vec3(0);
        jacobian[0].a.ang =  bodyA.orientation * glm::vec3(1, 0, 0);
        jacobian[0].b.ang = -bodyA.orientation * glm::vec3(1, 0, 0);

        jacobian[1].a.lin = glm::vec3(0);
        jacobian[1].b.lin = glm::vec3(0);
        jacobian[1].a.ang =  bodyA.orientation * glm::vec3(0, 1, 0);
        jacobian[1].b.ang = -bodyA.orientation * glm::vec3(0, 1, 0);

        jacobian[2].a.lin = glm::vec3(0);
        jacobian[2].b.lin = glm::vec3(0);
        jacobian[2].a.ang =  bodyA.orientation * glm::vec3(0, 0, 1);
        jacobian[2].b.ang = -bodyA.orientation * glm::vec3(0, 0, 1);

        glm::vec3 relAngVel = useIntegratedVelocities
            ? glm::transpose(bodyA.orientation) * (bodyB.integratedAngularVelocity - bodyA.integratedAngularVelocity)
            : glm::transpose(bodyA.orientation) * (bodyB.angularVelocity - bodyA.angularVelocity);

        ConstraintVariables::setReaction(velStage, targetAngularVelocity + relAngVel);
        ConstraintVariables::setMinImpulses(velStage, minTorque * dt);
        ConstraintVariables::setMaxImpulses(velStage, maxTorque * dt);

        ConstraintVariables::setReaction(posStage, glm::vec3(0));
        ConstraintVariables::setMinImpulses(posStage, glm::vec3(0));
        ConstraintVariables::setMaxImpulses(posStage, glm::vec3(0));
    }

    //
    // ConstraintCollision
    //
    void ConstraintCollision::buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) {
        useBlock[0] = false;
        useBlock[1] = false;
        useBlock[2] = false;

        glm::vec3 midPoint = pointA + 0.5f * (depth * normal);
        glm::vec3 relA = midPoint - bodyA.position;
        glm::vec3 relB = midPoint - bodyB.position;

        jacobian[0].a.lin = -normal;
        jacobian[0].b.lin = normal;
        jacobian[0].a.ang = -glm::cross(relA, normal);
        jacobian[0].b.ang = glm::cross(relB, normal);

        glm::vec3 intDeltaV = (bodyA.integratedLinearVelocity + glm::cross(bodyA.integratedAngularVelocity, relA)) - (bodyB.integratedLinearVelocity + glm::cross(bodyB.integratedAngularVelocity, relB));
        float intNormalVel = glm::dot(intDeltaV, normal);

        glm::vec3 prevDeltaV = (bodyA.linearVelocity + glm::cross(bodyA.angularVelocity, relA)) - (bodyB.linearVelocity + glm::cross(bodyB.angularVelocity, relB));
        float prevNormalVel = glm::dot(prevDeltaV, normal);

        velStage[0].reaction = intNormalVel + restitution * (prevNormalVel < config.collisionRestitutionThreshold ? 0.0f : prevNormalVel);
        velStage[0].minImpulseValue = 0.0f;
        velStage[0].maxImpulseValue = std::numeric_limits<float>::infinity();

        float penetrationMargin = config.collisionPenetrationMargin;

        glm::vec3 intTangVel = intDeltaV - intNormalVel * normal;
        glm::vec3 prevTangVel = prevDeltaV - prevNormalVel * normal;
        float prevTangSpeedSq = glm::dot(prevTangVel, prevTangVel);

        glm::vec3 t1, t2;
        float frictionBound;

        if (prevTangSpeedSq < config.collisionFrictionStaticToDynamicThreshold * config.collisionFrictionStaticToDynamicThreshold) {
            generateOrthonormalBasis(t1, t2, normal);
            frictionBound = config.collisionFrictionStaticScale * friction;

            glm::vec3 cachedT2 = glm::cross(cachedTangent1, normal);
            glm::vec3 velImpulse = velStage[1].impulse * cachedTangent1 + velStage[2].impulse * cachedT2;
            glm::vec3 posImpulse = posStage[1].impulse * cachedTangent1 + posStage[2].impulse * cachedT2;

            velStage[1].reaction = glm::dot(intTangVel, t1);
            velStage[1].impulse = glm::dot(velImpulse, t1);
            velStage[1].minImpulseValue = -frictionBound;
            velStage[1].maxImpulseValue = frictionBound;

            velStage[2].reaction = glm::dot(intTangVel, t2);
            velStage[2].impulse = glm::dot(velImpulse, t2);
            velStage[2].minImpulseValue = -frictionBound;
            velStage[2].maxImpulseValue = frictionBound;

            posStage[1].reaction = 0.0f;
            posStage[1].impulse = glm::dot(posImpulse, t1);
            posStage[1].minImpulseValue = -frictionBound;
            posStage[1].maxImpulseValue = frictionBound;

            posStage[2].reaction = 0.0f;
            posStage[2].impulse = glm::dot(posImpulse, t2);
            posStage[2].minImpulseValue = -frictionBound;
            posStage[2].maxImpulseValue = frictionBound;

            cachedTangent1 = t1;

            const float gravity = 196.2f;
            float dA = config.collisionPenetrationMarginMaxBumpProportions * (gravity + 0.1f) / glm::dot(bodyA.angularVelocity, bodyA.angularVelocity);
            float dB = config.collisionPenetrationMarginMaxBumpProportions * (gravity + 0.1f) / glm::dot(bodyB.angularVelocity, bodyB.angularVelocity);
            penetrationMargin = std::max(config.collisionPenetrationMarginMin, std::min(config.collisionPenetrationMarginMax, std::min(dA, dB)));
        } else {
            t1 = glm::normalize(prevTangVel);
            t2 = glm::cross(t1, normal);
            frictionBound = config.collisionFrictionDynamicScale * friction;

            glm::vec3 cachedT2 = glm::cross(cachedTangent1, normal);
            glm::vec3 velImpulse = velStage[1].impulse * cachedTangent1 + velStage[2].impulse * cachedT2;

            velStage[1].reaction = glm::dot(intTangVel, t1);
            velStage[1].impulse = glm::dot(velImpulse, t1);
            velStage[1].minImpulseValue = 0.0f;
            velStage[1].maxImpulseValue = frictionBound;

            velStage[2].reaction = 0.0f;
            velStage[2].impulse = 0.0f;
            velStage[2].minImpulseValue = 0.0f;
            velStage[2].maxImpulseValue = 0.0f;

            posStage[1] = ConstraintVariables();
            posStage[2] = ConstraintVariables();
            posStage[1].minImpulseValue = 0.0f;
            posStage[1].maxImpulseValue = 0.0f;
            posStage[2].minImpulseValue = 0.0f;
            posStage[2].maxImpulseValue = 0.0f;

            cachedTangent1 = t1;

            float dynamicVelocityExcess = std::min(config.collisionPenetrationVelocityForMinMargin, std::sqrt(prevTangSpeedSq) - config.collisionFrictionStaticToDynamicThreshold);
            float tBlend = dynamicVelocityExcess / (config.collisionPenetrationVelocityForMinMargin - config.collisionFrictionStaticToDynamicThreshold);
            penetrationMargin = tBlend * config.collisionPenetrationMarginMin + (1.0f - tBlend) * config.collisionPenetrationMarginMax;
        }

        jacobian[1].a.lin = -t1;
        jacobian[1].b.lin = t1;
        jacobian[1].a.ang = -glm::cross(relA, t1);
        jacobian[1].b.ang = glm::cross(relB, t1);

        jacobian[2].a.lin = -t2;
        jacobian[2].b.lin = t2;
        jacobian[2].a.ang = -glm::cross(relA, t2);
        jacobian[2].b.ang = glm::cross(relB, t2);

        posStage[0].reaction = -config.collisionPenetrationResolutionDamping * (depth + penetrationMargin);
        posStage[0].minImpulseValue = 0.0f;
        posStage[0].maxImpulseValue = std::numeric_limits<float>::infinity();
    }

    ConstraintCollision::Convergence ConstraintCollision::testPGSConvergence(const float* disp, const float* residuals, const float* deltaResiduals, const SolverConfig& config) {
        float base = config.inconsistentConstraintCollisionBaseThreshold;
        float threshold = config.inconsistentConstraintCollisionThreshold;

        if (-base < residuals[0] && residuals[0] < threshold) {
            if (std::abs(deltaResiduals[0]) < config.inconsistentConstraintDeltaThreshold * threshold) {
                return Convergence_Converges;
            }
        }
        if (residuals[0] >= threshold) {
            if (deltaResiduals[0] > -config.inconsistentConstraintDeltaThreshold * residuals[0]) {
                return Convergence_Diverges;
            }
        }
        return Convergence_Undetermined;
    }

    //
    // ConstraintLinearSpring
    //
    // BUG 3 FIX: original uses Constants::worldDt() (the actual dt), not a
    // hardcoded 1/240. Pass-through dt from buildEquation.
    void ConstraintLinearSpring::buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) {
        useBlock[0] = 0;
        useBlock[1] = 0;
        useBlock[2] = 0;

        glm::vec3 worldPointA = bodyA.orientation * pivotA + bodyA.position;
        glm::vec3 worldPointB = bodyB.orientation * pivotB + bodyB.position;

        glm::vec3 relA = worldPointA - bodyA.position;
        glm::vec3 relB = worldPointB - bodyB.position;

        jacobian[0].a.lin = glm::vec3( 1, 0, 0);
        jacobian[0].b.lin = glm::vec3(-1, 0, 0);
        jacobian[0].a.ang = -glm::vec3(0, -relA.z, relA.y);
        jacobian[0].b.ang = glm::vec3(0, -relB.z, relB.y);

        jacobian[1].a.lin = glm::vec3( 0, 1, 0);
        jacobian[1].b.lin = glm::vec3( 0,-1, 0);
        jacobian[1].a.ang = -glm::vec3( relA.z, 0, -relA.x);
        jacobian[1].b.ang = glm::vec3( relB.z, 0, -relB.x);

        jacobian[2].a.lin = glm::vec3( 0, 0, 1);
        jacobian[2].b.lin = glm::vec3( 0, 0,-1);
        jacobian[2].a.ang = -glm::vec3(-relA.y, relA.x, 0);
        jacobian[2].b.ang = glm::vec3(-relB.y, relB.x, 0);

        // Note: order is A - B here (opposite of ConstraintBallInSocket's B - A) btw
        glm::vec3 vA = bodyA.integratedLinearVelocity + glm::cross(bodyA.integratedAngularVelocity, relA);
        glm::vec3 vB = bodyB.integratedLinearVelocity + glm::cross(bodyB.integratedAngularVelocity, relB);
        glm::vec3 deltaV = vA - vB;
        glm::vec3 deltaP = worldPointA - worldPointB;

        // BUG 3 FIX: was (1.0f / 240.0f), now uses dt parameter
        glm::vec3 reaction = -(p * deltaP + d * deltaV) * dt;

        ConstraintVariables::setReaction(velStage, reaction);
        ConstraintVariables::setMinImpulses(velStage, -maxForce * dt);
        ConstraintVariables::setMaxImpulses(velStage,  maxForce * dt);

        ConstraintVariables::setReaction(posStage, glm::vec3(0));
        ConstraintVariables::setMinImpulses(posStage, glm::vec3(0));
        ConstraintVariables::setMaxImpulses(posStage, glm::vec3(0));
    }

    //
    // ConstraintAngularVelocity
    //
    void ConstraintAngularVelocity::buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) {
        glm::mat3 rB = getBodyB() ? getBodyB()->getWorldCFrame().rotation : glm::mat3(1.0f);
        glm::vec3 worldSpaceAxisB = rB * axisB;

        jacobian[0].a.lin = glm::vec3(0);
        jacobian[0].b.lin = glm::vec3(0);
        jacobian[0].a.ang = worldSpaceAxisB;
        jacobian[0].b.ang = -worldSpaceAxisB;

        glm::vec3 relAngularVel = bodyB.integratedAngularVelocity - bodyA.integratedAngularVelocity;
        float rotVel = glm::dot(worldSpaceAxisB, relAngularVel) - desiredAngularVelocity;
        float maxImpulse = maxForce * dt;

        velStage[0].maxImpulseValue = maxImpulse;
        velStage[0].minImpulseValue = -maxImpulse;
        velStage[0].reaction = rotVel;

        posStage[0].maxImpulseValue = 0.0f;
        posStage[0].minImpulseValue = 0.0f;
        posStage[0].reaction = 0.0f;
    }
} // namespace OWSolver
