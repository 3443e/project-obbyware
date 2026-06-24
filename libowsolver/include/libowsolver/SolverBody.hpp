#pragma once
#include "libowsolver/Simd.hpp"
#include <glm/glm.hpp>
#include <cstdint>

namespace OWSolver {
    struct SolverBodyDynamicProperties {
        glm::vec3 integratedLinearVelocity;
        glm::vec3 integratedAngularVelocity;
        glm::mat3 orientation;
        glm::vec3 position;
        glm::vec3 linearVelocity;
        glm::vec3 angularVelocity;
    };

    struct SolverBodyStaticProperties {
        uint64_t bodyUID;
        uint32_t guid;
        bool isStatic;
    };

    
    struct SymmetricMatrix {
        // [ d0 a  b ]
        // [ a  d1 c ]
        // [ b  c  d2]
        glm::vec3 diagonals; // [d0, d1, d2]
        glm::vec3 offDiagonals; // [a,  b,  c ]

        glm::vec3 operator*(const glm::vec3& v) const {
            return glm::vec3(
                diagonals.x * v.x + offDiagonals.x * v.y + offDiagonals.y * v.z,
                offDiagonals.x * v.x + diagonals.y * v.y + offDiagonals.z * v.z,
                offDiagonals.y * v.x + offDiagonals.z * v.y + diagonals.z * v.z
            );
        }
    };

    // SolverBodyMassAndInertia
    struct SolverBodyMassAndInertia {
        glm::vec3 inertiaDiagonal;
        float massInvVelStage;
        glm::vec3 inertiaOffDiagonal;
        float posToVelMassRatio;

        float getInvMassVelStage() const {
            return massInvVelStage;
        }
        float getInvMassPosStage() const {
            return (massInvVelStage * posToVelMassRatio);
        }

        SymmetricMatrix getInvInertiaVelStage() const {
            return {inertiaDiagonal, inertiaOffDiagonal};
        }

        SymmetricMatrix getInvInertiaPosStage(float scale) const {
            float s = scale * posToVelMassRatio;
            return {s * inertiaDiagonal, s * inertiaOffDiagonal};
        }
    };

} // namespace OWSolver
