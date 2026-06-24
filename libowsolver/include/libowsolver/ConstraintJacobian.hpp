#pragma once
#include "libowsolver/Simd.hpp"
#include "libowsolver/SolverBody.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace OWSolver {
    // body pair indices, which two bodies a constraint connects
    struct BodyPairIndices {
        int first;
        int second;
        BodyPairIndices() : first(0), second(0) {}
        BodyPairIndices(int a, int b) : first(a), second(b) {}
    };

    // virtual displacement stores a linear and angular vec3, used for:
    struct VirtualDisplacement {
        glm::vec3 lin;
        glm::vec3 ang;

        VirtualDisplacement() : lin(0), ang(0) {}
        VirtualDisplacement(const glm::vec3& l, const glm::vec3& a) : lin(l), ang(a) {}

        void reset() {
            lin = glm::vec3(0);
            ang = glm::vec3(0);
        }
    };

    // array of virtual displacements
    struct VirtualDisplacementArray {
        std::vector<VirtualDisplacement> data;

        explicit VirtualDisplacementArray(size_t size) : data(size) {}

        void reset() {
            for (auto& v : data) {
                v.reset();
            }
        }

        VirtualDisplacement& operator[](int i){
            return data[i];
        }

        const VirtualDisplacement& operator[](int i) const {
            return data[i];
        }

        size_t size() const {
            return data.size();
        }
    };

    // Effective mass: W * J^t for one body
    struct EffectiveMass {
        glm::vec3 lin;
        glm::vec3 ang;

        EffectiveMass() : lin(0), ang(0) {}
        EffectiveMass(const glm::vec3& l, const glm::vec3& a) : lin(l), ang(a) {}

        void reset() {
            lin = glm::vec3(0);
            ang = glm::vec3(0);
        }

        void applyMultiplier(float m) {
            lin *= m;
            ang *= m;
        }
    };

    // effective mass pair struct, for 2 bodies
    struct EffectiveMassPair {
        EffectiveMass a;
        EffectiveMass b;

        EffectiveMassPair() {}
        EffectiveMassPair(const EffectiveMass& ea, const EffectiveMass& eb) : a(ea), b(eb) {}

        void reset() {
            a.reset();
            b.reset();
        }

        void applyMultipliers(float mA, float mB) {
            a.applyMultiplier(mA);
            b.applyMultiplier(mB);
        }

        const EffectiveMass& getPartA() const {
            return a;
        }

        const EffectiveMass& getPartB() const {
            return b;
        }
    };

    struct ConstraintJacobian {
        glm::vec3 lin;
        glm::vec3 ang;

        ConstraintJacobian() : lin(0), ang(0) {}
        void reset() {
            lin = glm::vec3(0);
            ang = glm::vec3(0);
        }
    };

    // full Jacobian pair for a binary constraint (body A and body B)
    struct ConstraintJacobianPair {
        ConstraintJacobian a; // body A
        ConstraintJacobian b; // body B

        void reset() {
            a.reset();
            b.reset();
        }

        // equivalent to J * W * J^t for this row
        float dot(const EffectiveMassPair& em) const {
            float r = 0;
            r += glm::dot(a.lin, em.a.lin);
            r += glm::dot(a.ang, em.a.ang);
            r += glm::dot(b.lin, em.b.lin);
            r += glm::dot(b.ang, em.b.ang);
            return r;
        }
    };

    // helper functions ahead:

    // helper: apply an impulse to a virtual displacement
    // virD += impulse * effectiveMass
    inline VirtualDisplacement applyImpulse(const VirtualDisplacement& virD, float impulse, const EffectiveMass& eff) {
        return VirtualDisplacement(virD.lin + impulse * eff.lin, virD.ang + impulse * eff.ang);
    }

    // helper: project virtual displacement onto a jacobian
    // returns scalar: J dot virD
    inline float projectOntoJacobian(const ConstraintJacobianPair& j, const VirtualDisplacement& va, const VirtualDisplacement& vb) {
        return glm::dot(j.a.lin, va.lin) + glm::dot(j.a.ang, va.ang) + glm::dot(j.b.lin, vb.lin) + glm::dot(j.b.ang, vb.ang);
    }

} // namespace OWSolver
