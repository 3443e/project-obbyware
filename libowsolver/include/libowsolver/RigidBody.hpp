#pragma once
#include <glm/glm.hpp>
#include "libowsolver/CoordinateFrame.hpp"

namespace OWSolver {
    class RigidBody {
    public:
        RigidBody() = default;

        void setMass(float m) { mass = m; massInv = (m > 0.0f) ? (1.0f / m) : 0.0f; }
        void setMomentOfInertia(const glm::vec3& diagonal) {
            inertiaBodyDiagonal = diagonal;
            inertiaBodyInvDiagonal = glm::vec3(
                diagonal.x > 0.0f ? 1.0f / diagonal.x : 0.0f,
                diagonal.y > 0.0f ? 1.0f / diagonal.y : 0.0f,
                diagonal.z > 0.0f ? 1.0f / diagonal.z : 0.0f);
        }

        void setBoxMassAndInertia(float massValue, const glm::vec3& size) {
            setMass(massValue);
            glm::vec3 sq = size * size;
            glm::vec3 diag(
                (massValue / 12.0f) * (sq.y + sq.z),
                (massValue / 12.0f) * (sq.x + sq.z),
                (massValue / 12.0f) * (sq.x + sq.y)
            );
            setMomentOfInertia(diag);
        }

        glm::vec3 position{0.0f};
        glm::mat3 orientation{1.0f};
        glm::vec3 linearVelocity{0.0f};
        glm::vec3 angularVelocity{0.0f};

        CoordinateFrame getCoordinateFrame() const { return CoordinateFrame(orientation, position); }
        void setCoordinateFrame(const CoordinateFrame& cf) { orientation = cf.rotation; position = cf.translation; }

        float mass = 1.0f;
        float massInv = 1.0f;
        glm::vec3 inertiaBodyDiagonal{1.0f};
        glm::vec3 inertiaBodyInvDiagonal{1.0f};

        glm::mat3 getInverseInertiaWorld() const {
            glm::mat3 R = orientation;
            glm::mat3 invIBody(0.0f);
            invIBody[0][0] = inertiaBodyInvDiagonal.x;
            invIBody[1][1] = inertiaBodyInvDiagonal.y;
            invIBody[2][2] = inertiaBodyInvDiagonal.z;
            return R * invIBody * glm::transpose(R);
        }

        glm::vec3 force{0.0f};
        glm::vec3 torque{0.0f};
        glm::vec3 impulse{0.0f};
        glm::vec3 rotationalImpulse{0.0f};

        bool isStatic = false;

        void clearForceAccumulators() { force = glm::vec3(0.0f); torque = glm::vec3(0.0f); }
        void clearImpulseAccumulators() { impulse = glm::vec3(0.0f); rotationalImpulse = glm::vec3(0.0f); }

        void accumulateForceAtBranchCofm(const glm::vec3& f) {
            if (isStatic) return;
            force += f;
        }

        void accumulateForce(const glm::vec3& f, const glm::vec3& worldPos) {
            if (isStatic) return;
            force += f;
            glm::vec3 localPosWorld = worldPos - position;
            torque += glm::cross(localPosWorld, f);
        }

        void accumulateTorque(const glm::vec3& t) {
            if (isStatic) return;
            torque += t;
        }

        void accumulateImpulseAtBranchCofm(const glm::vec3& i) {
            if (isStatic) return;
            impulse += i;
        }

        void accumulateLinearImpulse(const glm::vec3& i, const glm::vec3& worldPos) {
            if (isStatic) return;
            impulse += i;
            glm::vec3 localPosWorld = worldPos - position;
            rotationalImpulse += glm::cross(localPosWorld, i);
        }

        void accumulateRotationalImpulse(const glm::vec3& t) {
            if (isStatic) return;
            rotationalImpulse += t;
        }

        glm::vec3 getBranchForce() const { return force; }
        glm::vec3 getBranchTorque() const { return torque; }
        glm::vec3 getBranchVelocity() const { return linearVelocity; }
        float getBranchMass() const { return mass; }
        glm::vec3 getBranchIBodyV3() const { return inertiaBodyDiagonal; }
    };

} // namespace OWSolver
