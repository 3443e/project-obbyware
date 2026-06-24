#pragma once
// this is from legacy spring-joint system, run every kernel substep (19x per world step)
#include "libowsolver/RigidBody.hpp"
#include "libowsolver/CoordinateFrame.hpp"

namespace OWSolver {
    class Motor6D {
    public:
        Motor6D(RigidBody* bodyA, RigidBody* bodyB) : bodyA(bodyA), bodyB(bodyB) {}

        void setC0(const CoordinateFrame& c) {
            j0 = c;
        }

        void setC1(const CoordinateFrame& c) {
            j1 = c;
        }

        // like RotateConnector ctor: k = kValue * armLength^2
        // armLength here should be the limb's half-extent from the joint to its COM,
        void setStiffness(float kValue, float armLength) {
            k = kValue * armLength * armLength;
        }

        // animation stuff i think Motor::desiredAngle / Motor6D pose.rotaxisangle.z
        void setDesiredAngle(float angle) {
            desiredAngle = angle;
        }

        float getDesiredAngle() const {
            return desiredAngle;
        }

        float getCurrentAngle() const {
            return currentAngle;
        }

        void setBaseAngle(float angle) { baseAngle = angle; }

        // call once after C0/C1 are set and bodies are at rest pose, to capture baseAngle.
        void resetToCurrentAsBase() {
            glm::vec3 normal;
            float angle = computeJointAngle(normal);
            baseAngle = angle;
        }

        // irrors RotateConnector::computeForce(throttling), called every kernel substep.
        void computeForce() {
            glm::vec3 normal;
            currentAngle = computeNormalRotationFromBase(normal);
            float deltaRotation = deltaRotationClose(desiredAngle, currentAngle);

            float torqueVal = k * deltaRotation;
            glm::vec3 torque = normal * torqueVal;

            bodyA->accumulateTorque(-torque);
            bodyB->accumulateTorque(torque);
        }

    private:
        RigidBody* bodyA;
        RigidBody* bodyB;
        CoordinateFrame j0; // joint coord in bodyA's object space
        CoordinateFrame j1; // joint coord in bodyB's object space

        float k = 0.0f;
        float baseAngle = 0.0f;
        float desiredAngle = 0.0f;
        float currentAngle = 0.0f;

        float computeJointAngle(glm::vec3& normalOut) const {
            glm::mat3 j0World = bodyA->orientation * j0.rotation;
            glm::mat3 j1World = bodyB->orientation * j1.rotation;
            glm::mat3 j1Inj0 = glm::transpose(j0World) * j1World;
            normalOut = j0World[2]; // column(2)
            return zAxisAngle(j1Inj0);
        }

        float computeNormalRotationFromBase(glm::vec3& normalOut) const {
            float angle = computeJointAngle(normalOut);
            return angle - baseAngle;
        }
    };

} // namespace OWSolver
