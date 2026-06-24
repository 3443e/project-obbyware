#pragma once
#include "libowsolver/Constraint.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <limits>
#include <cmath>

namespace OWSolver {
    //
    // ConstraintBallInSocket
    //
    class ConstraintBallInSocket : public Constraint {
    public:
        ConstraintBallInSocket(Body* a, Body* b) : Constraint(Types_BallInSocket, a, b, 3) {}

        void setPivotA(const glm::vec3& p) {
            pointA = p;
        }

        void setPivotB(const glm::vec3& p) {
            pointB = p;
        }

        Convergence testPGSConvergence(const float* disp, const float* residuals, const float* deltaResiduals, const SolverConfig& config) override;

    private:
        void buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) override;

        glm::vec3 pointA;
        glm::vec3 pointB;
    };

    //
    // ConstraintAlign2Axes
    //
    class ConstraintAlign2Axes : public Constraint {
    public:
        ConstraintAlign2Axes(Body* a, Body* b) : Constraint(Types_Align2Axes, a, b, 2) , worldSpaceOrthogonalB1(0) , worldSpaceOrthogonalB2(0) {}

        void setAxisA(const glm::vec3& a);
        void setAxisB(const glm::vec3& b);

        Convergence testPGSConvergence(const float* disp, const float* residuals, const float* deltaResiduals, const SolverConfig& config) override;

    private:
        void buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) override;

        glm::vec3 axisA;
        glm::vec3 axisB;
        glm::vec3 orthogonalAxisB1;
        glm::vec3 orthogonalAxisB2;

        // cached world space values
        glm::vec3 worldSpaceOrthogonalB1;
        glm::vec3 worldSpaceOrthogonalB2;
    };

    //
    // ConstraintLinearVelocity
    //
    class ConstraintLinearVelocity : public Constraint {
    public:
        ConstraintLinearVelocity(Body* a, Body* b) : Constraint(Types_LinearVelocity, a, b, 3) , maxForce(0) , desiredVelocity(0) {}

        void setDesiredVelocity(const glm::vec3& v) {
            desiredVelocity = v;
        }
        void setMaxForce(const glm::vec3& f) {
            maxForce = f;
        }
    private:
        void buildEquation( ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) override;

        glm::vec3 maxForce;
        glm::vec3 desiredVelocity;
    };

    //
    // ConstraintBodyAngularVelocity
    //
    class ConstraintBodyAngularVelocity : public Constraint {
    public:
        ConstraintBodyAngularVelocity(Body* a, Body* b) : Constraint(Types_BodyAngularVelocity, a, b, 3) , targetAngularVelocity(0) , maxTorque(0) , minTorque(0) , useIntegratedVelocities(false) {}

        void setTarget(const glm::vec3& v){
            targetAngularVelocity = v;
        }

        void setMaxTorque(const glm::vec3& v) {
            maxTorque = v;
        }
        void setMinTorque(const glm::vec3& v) {
            minTorque = v;
        }
        void setUseIntegratedVelocities(bool b){
            useIntegratedVelocities = b;
        }

    private:
        void buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) override;

        glm::vec3 targetAngularVelocity;
        glm::vec3 maxTorque;
        glm::vec3 minTorque;
        bool useIntegratedVelocities;
    };

    //
    // ConstraintCollision
    //
    // BUG 1 FIX: original Constraint.h ~line 4806 explicitly resets the SOR
    // values to 1.0 for all 3 cache rows. The default-constructed ConstraintCache
    // uses velocitySor=1.9 / positionSor=1.9 which causes first-frame explosions
    // when the contact is fresh. Mirror the original ctor exactly.
    class ConstraintCollision : public Constraint {
    public:
        ConstraintCollision(Body* a, Body* b) : Constraint(Types_Collision, a, b, 3) , depth(0) , friction(0), restitution(0) , cachedTangent1(1, 0, 0) {
            getCache(0).velocitySor = 1.0f;
            getCache(0).positionSor = 1.0f;
            getCache(1).velocitySor = 1.0f;
            getCache(1).positionSor = 1.0f;
            getCache(2).velocitySor = 1.0f;
            getCache(2).positionSor = 1.0f;
        }

        void setNormal(const glm::vec3& n) {
            normal = n;
        }
        void setPointA(const glm::vec3& p) {
            pointA = p;
        }
        void setDepth(float d) {
            depth = d;
        }
        void setFriction(float f) {
            friction = f;
        }
        void setRestitution(float r) {
            restitution = r;
        }

        glm::vec3 getNormal() const {
            return normal;
        }

        glm::vec3 getPointA() const {
            return pointA;
        }

        float getDepth() const {
            return depth;
        }

        float getFriction() const {
            return friction;
        }

        float getRestitution() const {
            return restitution;
        }

        Convergence testPGSConvergence(const float* disp, const float* residuals, const float* deltaResiduals, const SolverConfig& config) override;
    private:
        void buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) override;

        glm::vec3 normal;
        glm::vec3 pointA;
        float depth;
        float friction;
        float restitution;
        glm::vec3 cachedTangent1;
    };

    //
    // ConstraintLinearSpring
    //
    class ConstraintLinearSpring : public Constraint {
    public:
        ConstraintLinearSpring(Body* a, Body* b) : Constraint(Types_LinearSpring, a, b, 3) , pivotA(0), pivotB(0), maxForce(0), p(0), d(0) {}

        void setPivotA(const glm::vec3& v) {
            pivotA = v; 
        }

        void setPivotB(const glm::vec3& v) {
            pivotB = v;
        }

        void setMaxForce(const glm::vec3& f) {
            maxForce = f;
        }

        void setPD(float pVal, float dVal) {
            p = pVal; d = dVal;
        }

    private:
        void buildEquation(ConstraintJacobianPair* jacobian, uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) override;

        glm::vec3 pivotA;
        glm::vec3 pivotB;
        glm::vec3 maxForce;
        float p, d;
    };
} // namespace OWSolver
