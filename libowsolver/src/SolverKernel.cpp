#include "libowsolver/SolverKernel.hpp"
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace OWSolver {
    static float invertBlock1(float d) {
        return d > 1e-30f ? 1.0f / d : 0.0f;
    }

    // helper
    // [ d0 d01 ]
    // [ d01 d1 ]
    struct Block2 {
        float m00, m01, m11;
    };
    static Block2 invertBlock2(float d0, float d01, float d1) {
        float det = d0 * d1 - d01 * d01;
        if (std::abs(det) < 1e-30f) return {0, 0, 0};
        float inv = 1.0f / det;
        return { d1 * inv, -d01 * inv, d0 * inv };
    }

    // helper
    struct Block3 {
        float m[3][3];
    };
    static Block3 invertBlock3(float d0, float d1, float d2, float a, float b, float c) {
        // d0=diag0, d1=diag1, d2=diag2
        // a=off01, b=off02, c=off12
        float det = d0*(d1*d2 - c*c) - a*(a*d2 - c*b) + b*(a*c - d1*b);
        Block3 r;
        if (std::abs(det) < 1e-30f) {
            for (int i=0;i<3;i++) for (int j=0;j<3;j++) r.m[i][j]=0;
            return r;
        }
        float inv = 1.0f / det;
        r.m[0][0] = (d1*d2 - c*c) * inv;
        r.m[1][1] = (d0*d2 - b*b) * inv;
        r.m[2][2] = (d0*d1 - a*a) * inv;
        r.m[0][1] = r.m[1][0] = -(a*d2 - b*c) * inv;
        r.m[0][2] = r.m[2][0] = (a*c - b*d1) * inv;
        r.m[1][2] = r.m[2][1] = -(d0*c - b*a) * inv;
        return r;
    }

    // returns J_i dot (virDA, virDB)
    static float projectVirD(const ConstraintJacobianPair& j, const VirtualDisplacement& va, const VirtualDisplacement& vb) {
        return glm::dot(j.a.lin, va.lin) + glm::dot(j.a.ang, va.ang) + glm::dot(j.b.lin, vb.lin) + glm::dot(j.b.ang, vb.ang);
    }

    static void addImpulseToVirD(VirtualDisplacement& virD, float dImpulse, const EffectiveMass& eff) {
        virD.lin += dImpulse * eff.lin;
        virD.ang += dImpulse * eff.ang;
    }

    // update a single 1D constraint (dim=1)
    static void updateConstraint1(
        ConstraintVariables& vel,
        ConstraintVariables& pos,
        VirtualDisplacement& virDVelA, VirtualDisplacement& virDVelB,
        VirtualDisplacement& virDPosA, VirtualDisplacement& virDPosB,
        const ConstraintJacobianPair& preJacVel,
        const ConstraintJacobianPair& preJacPos,
        const EffectiveMassPair& effVel,
        const EffectiveMassPair& effPos
    ) {
        float projVel = projectVirD(preJacVel, virDVelA, virDVelB);
        float projPos = projectVirD(preJacPos, virDPosA, virDPosB);

        float oldVel = vel.impulse;
        float oldPos = pos.impulse;

        float newVel = std::min(std::max(oldVel + vel.reaction - projVel, vel.minImpulseValue), vel.maxImpulseValue);
        float newPos = std::min(std::max(oldPos + pos.reaction - projPos, pos.minImpulseValue), pos.maxImpulseValue);

        float dVel = newVel - oldVel;
        float dPos = newPos - oldPos;

        vel.impulse = newVel;
        pos.impulse = newPos;

        addImpulseToVirD(virDVelA, dVel, effVel.getPartA());
        addImpulseToVirD(virDVelB, dVel, effVel.getPartB());
        addImpulseToVirD(virDPosA, dPos, effPos.getPartA());
        addImpulseToVirD(virDPosB, dPos, effPos.getPartB());
    }

    // Update a 2D constraint block
    static void updateConstraint2(
        ConstraintVariables* vel,
        ConstraintVariables* pos,
        VirtualDisplacement& virDVelA, VirtualDisplacement& virDVelB,
        VirtualDisplacement& virDPosA, VirtualDisplacement& virDPosB,
        const ConstraintJacobianPair* preJacVel,
        const ConstraintJacobianPair* preJacPos,
        const EffectiveMassPair* effVel,
        const EffectiveMassPair* effPos
    ) {
        float pv0 = projectVirD(preJacVel[0], virDVelA, virDVelB);
        float pv1 = projectVirD(preJacVel[1], virDVelA, virDVelB);
        float pp0 = projectVirD(preJacPos[0], virDPosA, virDPosB);
        float pp1 = projectVirD(preJacPos[1], virDPosA, virDPosB);

        float oldV0 = vel[0].impulse, oldV1 = vel[1].impulse;
        float oldP0 = pos[0].impulse, oldP1 = pos[1].impulse;

        // compute new impulses with block preconditioning already applied
        float newV0 = oldV0 + vel[0].reaction - pv0;
        float newV1 = oldV1 + vel[1].reaction - pv1;
        float newP0 = oldP0 + pos[0].reaction - pp0;
        float newP1 = oldP1 + pos[1].reaction - pp1;

        newV0 = std::min(std::max(newV0, vel[0].minImpulseValue), vel[0].maxImpulseValue);
        newV1 = std::min(std::max(newV1, vel[1].minImpulseValue), vel[1].maxImpulseValue);
        newP0 = std::min(std::max(newP0, pos[0].minImpulseValue), pos[0].maxImpulseValue);
        newP1 = std::min(std::max(newP1, pos[1].minImpulseValue), pos[1].maxImpulseValue);

        float dV0 = newV0 - oldV0, dV1 = newV1 - oldV1;
        float dP0 = newP0 - oldP0, dP1 = newP1 - oldP1;

        vel[0].impulse = newV0; vel[1].impulse = newV1;
        pos[0].impulse = newP0; pos[1].impulse = newP1;

        addImpulseToVirD(virDVelA, dV0, effVel[0].getPartA());
        addImpulseToVirD(virDVelA, dV1, effVel[1].getPartA());
        addImpulseToVirD(virDVelB, dV0, effVel[0].getPartB());
        addImpulseToVirD(virDVelB, dV1, effVel[1].getPartB());
        addImpulseToVirD(virDPosA, dP0, effPos[0].getPartA());
        addImpulseToVirD(virDPosA, dP1, effPos[1].getPartA());
        addImpulseToVirD(virDPosB, dP0, effPos[0].getPartB());
        addImpulseToVirD(virDPosB, dP1, effPos[1].getPartB());
    }

    static void updateConstraint3(
        ConstraintVariables* vel,
        ConstraintVariables* pos,
        VirtualDisplacement& virDVelA, VirtualDisplacement& virDVelB,
        VirtualDisplacement& virDPosA, VirtualDisplacement& virDPosB,
        const ConstraintJacobianPair* preJacVel,
        const ConstraintJacobianPair* preJacPos,
        const EffectiveMassPair* effVel,
        const EffectiveMassPair* effPos,
        bool isCollision)
    {
        auto solve3 = [&](ConstraintVariables* vars, VirtualDisplacement& virDA, VirtualDisplacement& virDB, const ConstraintJacobianPair* preJac, const EffectiveMassPair* eff) {
            float p0 = projectVirD(preJac[0], virDA, virDB);
            float p1 = projectVirD(preJac[1], virDA, virDB);
            float p2 = projectVirD(preJac[2], virDA, virDB);

            float old0 = vars[0].impulse;
            float old1 = vars[1].impulse;
            float old2 = vars[2].impulse;

            float min0 = vars[0].minImpulseValue;
            float max0 = vars[0].maxImpulseValue;
            float min1 = vars[1].minImpulseValue;
            float max1 = vars[1].maxImpulseValue;
            float min2 = vars[2].minImpulseValue;
            float max2 = vars[2].maxImpulseValue;

            if (isCollision) {
                float normalImpulse = old0;
                min1 *= normalImpulse; max1 *= normalImpulse;
                min2 *= normalImpulse; max2 *= normalImpulse;
            }

            float n0 = std::min(std::max(old0 + vars[0].reaction - p0, min0), max0);
            float n1 = std::min(std::max(old1 + vars[1].reaction - p1, min1), max1);
            float n2 = std::min(std::max(old2 + vars[2].reaction - p2, min2), max2);

            float d0 = n0-old0, d1 = n1-old1, d2 = n2-old2;

            vars[0].impulse = n0;
            vars[1].impulse = n1;
            vars[2].impulse = n2;

            addImpulseToVirD(virDA, d0, eff[0].getPartA());
            addImpulseToVirD(virDA, d1, eff[1].getPartA());
            addImpulseToVirD(virDA, d2, eff[2].getPartA());
            addImpulseToVirD(virDB, d0, eff[0].getPartB());
            addImpulseToVirD(virDB, d1, eff[1].getPartB());
            addImpulseToVirD(virDB, d2, eff[2].getPartB());
        };

        solve3(vel, virDVelA, virDVelB, preJacVel, effVel);
        solve3(pos, virDPosA, virDPosB, preJacPos, effPos);
    }

    void ComputeEffectiveMasses(
        std::vector<EffectiveMassPair>& effVel,
        std::vector<EffectiveMassPair>& effPos,
        const std::vector<ConstraintJacobianPair>& jacobians,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        const std::vector<SolverBodyMassAndInertia>& massAndInertia,
        const SolverConfig& config
    ) {
        size_t offset = 0;
        for (size_t c = 0; c < dimensions.size(); c++) {
            uint8_t d = dimensions[c];
            const SolverBodyMassAndInertia& mA = massAndInertia[pairs[c].first];
            const SolverBodyMassAndInertia& mB = massAndInertia[pairs[c].second];

            for (uint8_t i = 0; i < d; i++) {
                const ConstraintJacobianPair& jac = jacobians[offset + i];

                // velocity stage: W * J^t using full mass
                EffectiveMass emAVel(mA.getInvMassVelStage() * jac.a.lin, mA.getInvInertiaVelStage() * jac.a.ang);
                EffectiveMass emBVel(mB.getInvMassVelStage() * jac.b.lin, mB.getInvInertiaVelStage() * jac.b.ang);
                effVel[offset + i] = EffectiveMassPair(emAVel, emBVel);

                // postion stage: use reduced mass for stabilization
                EffectiveMass emAPos(mA.getInvMassPosStage() * jac.a.lin, mA.getInvInertiaPosStage(config.stabilizationInertiaScale) * jac.a.ang);
                EffectiveMass emBPos(mB.getInvMassPosStage() * jac.b.lin, mB.getInvInertiaPosStage(config.stabilizationInertiaScale) * jac.b.ang);
                effPos[offset + i] = EffectiveMassPair(emAPos, emBPos);
            }
            offset += d;
        }
    }

    void ApplyEffectiveMassMultipliers(
        std::vector<EffectiveMassPair>& effVel,
        std::vector<EffectiveMassPair>& effPos,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        const std::vector<float>& multipliers,
        const SolverConfig& config
    ) {
        size_t offset = 0;
        for (size_t c = 0; c < dimensions.size(); c++) {
            uint8_t d = dimensions[c];
            float mA = multipliers[pairs[c].first];
            float mB = multipliers[pairs[c].second];
            for (uint8_t i = 0; i < d; i++) {
                effVel[offset+i].applyMultipliers(mA, mB);
                effPos[offset+i].applyMultipliers(mA, mB);
            }
            offset += d;
        }
    }

    void PreconditionConstraintEquations(
        std::vector<ConstraintJacobianPair>& preJacVel,
        std::vector<ConstraintJacobianPair>& preJacPos,
        std::vector<ConstraintVariables>& velStage,
        std::vector<ConstraintVariables>& posStage,
        const std::vector<ConstraintJacobianPair>& jacobians,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        const std::vector<uint8_t>& useBlock,
        const std::vector<float>& sorVel,
        const std::vector<float>& sorPos,
        const std::vector<EffectiveMassPair>& effVel,
        const std::vector<EffectiveMassPair>& effPos,
        const SolverConfig& config
    ) {
        size_t offset = 0;
        for (size_t c = 0; c < dimensions.size(); c++) {
            uint8_t d = dimensions[c];

            auto precondStage = [&](std::vector<ConstraintVariables>& stage, std::vector<ConstraintJacobianPair>& preJac, const std::vector<EffectiveMassPair>& eff, size_t sorOffset) {
                if (d == 1) {
                    float diag = jacobians[offset].dot(eff[offset]);
                    float inv = invertBlock1(diag);
                    float sor = sorVel[sorOffset];

                    preJac[offset].a.lin = (sor * inv) * jacobians[offset].a.lin;
                    preJac[offset].a.ang = (sor * inv) * jacobians[offset].a.ang;
                    preJac[offset].b.lin = (sor * inv) * jacobians[offset].b.lin;
                    preJac[offset].b.ang = (sor * inv) * jacobians[offset].b.ang;
                    stage[offset].reaction *= (sor * inv);

                } else if (d == 2) {
                    float d00 = jacobians[offset+0].dot(eff[offset+0]);
                    float d11 = jacobians[offset+1].dot(eff[offset+1]);
                    float d01 = useBlock[offset] && useBlock[offset+1] ? jacobians[offset+0].dot(eff[offset+1]) : 0.0f;

                    Block2 inv = invertBlock2(d00, d01, d11);
                    float s0 = sorVel[sorOffset+0];
                    float s1 = sorVel[sorOffset+1];

                    for (int row = 0; row < 2; row++) {
                        float r = (row==0) ? s0 : s1;
                        float p0 = (row==0) ? inv.m00 : inv.m01;
                        float p1 = (row==0) ? inv.m01 : inv.m11;

                        preJac[offset+row].a.lin = r*(p0*jacobians[offset+0].a.lin + p1*jacobians[offset+1].a.lin);
                        preJac[offset+row].a.ang = r*(p0*jacobians[offset+0].a.ang + p1*jacobians[offset+1].a.ang);
                        preJac[offset+row].b.lin = r*(p0*jacobians[offset+0].b.lin + p1*jacobians[offset+1].b.lin);
                        preJac[offset+row].b.ang = r*(p0*jacobians[offset+0].b.ang + p1*jacobians[offset+1].b.ang);
                    }

                    float r0 = stage[offset+0].reaction;
                    float r1 = stage[offset+1].reaction;
                    stage[offset+0].reaction = s0*(inv.m00*r0 + inv.m01*r1);
                    stage[offset+1].reaction = s1*(inv.m01*r0 + inv.m11*r1);

                } else if (d == 3) {
                    float d0 = jacobians[offset+0].dot(eff[offset+0]);
                    float d1 = jacobians[offset+1].dot(eff[offset+1]);
                    float d2 = jacobians[offset+2].dot(eff[offset+2]);
                    float a = (useBlock[offset+0]&&useBlock[offset+1]) ? jacobians[offset+0].dot(eff[offset+1]) : 0.0f;
                    float b = (useBlock[offset+0]&&useBlock[offset+2]) ? jacobians[offset+0].dot(eff[offset+2]) : 0.0f;
                    float cc = (useBlock[offset+1]&&useBlock[offset+2]) ? jacobians[offset+1].dot(eff[offset+2]) : 0.0f;

                    Block3 inv = invertBlock3(d0,d1,d2,a,b,cc);

                    for (int row = 0; row < 3; row++) {
                        float s = sorVel[sorOffset+row];
                        ConstraintJacobianPair pj;
                        pj.reset();
                        for (int col = 0; col < 3; col++) {
                            float p = inv.m[row][col];
                            pj.a.lin += p * jacobians[offset+col].a.lin;
                            pj.a.ang += p * jacobians[offset+col].a.ang;
                            pj.b.lin += p * jacobians[offset+col].b.lin;
                            pj.b.ang += p * jacobians[offset+col].b.ang;
                        }
                        pj.a.lin *= s; pj.a.ang *= s;
                        pj.b.lin *= s; pj.b.ang *= s;
                        preJac[offset+row] = pj;
                    }

                    float r0 = stage[offset+0].reaction;
                    float r1 = stage[offset+1].reaction;
                    float r2 = stage[offset+2].reaction;
                    for (int row = 0; row < 3; row++) {
                        float s = sorVel[sorOffset+row];
                        stage[offset+row].reaction = s*(inv.m[row][0]*r0 + inv.m[row][1]*r1 + inv.m[row][2]*r2);
                    }
                }
            };

            precondStage(velStage, preJacVel, effVel, offset);
            precondStage(posStage, preJacPos, effPos, offset);

            offset += d;
        }
    }

    void InitVirtualDisplacements(
        VirtualDisplacementArray& virDVel,
        VirtualDisplacementArray& virDPos,
        const std::vector<ConstraintVariables>& velStage,
        const std::vector<ConstraintVariables>& posStage,
        const std::vector<EffectiveMassPair>& effVel,
        const std::vector<EffectiveMassPair>& effPos,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        const SolverConfig& config
    ) {
        size_t offset = 0;
        for (size_t c = 0; c < dimensions.size(); c++) {
            uint8_t d = dimensions[c];
            int iA = pairs[c].first;
            int iB = pairs[c].second;

            for (uint8_t i = 0; i < d; i++) {
                float vi = velStage[offset+i].impulse;
                float pi = posStage[offset+i].impulse;
                addImpulseToVirD(virDVel[iA], vi, effVel[offset+i].getPartA());
                addImpulseToVirD(virDVel[iB], vi, effVel[offset+i].getPartB());
                addImpulseToVirD(virDPos[iA], pi, effPos[offset+i].getPartA());
                addImpulseToVirD(virDPos[iB], pi, effPos[offset+i].getPartB());
            }
            offset += d;
        }
    }

    // SolveKernel, main PGS loop
    void SolveKernel(
        std::vector<ConstraintVariables>& velStage,
        std::vector<ConstraintVariables>& posStage,
        VirtualDisplacementArray& virDVel,
        VirtualDisplacementArray& virDPos,
        const std::vector<ConstraintJacobianPair>& preJacVel,
        const std::vector<ConstraintJacobianPair>& preJacPos,
        const std::vector<EffectiveMassPair>& effVel,
        const std::vector<EffectiveMassPair>& effPos,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        size_t collisionCount,
        const SolverConfig& config
    ) {
        size_t constraintCount = dimensions.size();
        size_t pureCount = constraintCount - collisionCount;

        for (unsigned k = 0; k < config.pgsIterations; k++) {
            size_t offset = 0;
            for (size_t c = 0; c < pureCount; c++) {
                uint8_t d = dimensions[c];
                int iA = pairs[c].first;
                int iB = pairs[c].second;

                VirtualDisplacement& vaVel = virDVel[iA];
                VirtualDisplacement& vbVel = virDVel[iB];
                VirtualDisplacement& vaPos = virDPos[iA];
                VirtualDisplacement& vbPos = virDPos[iB];

                switch (d) {
                case 1:
                    updateConstraint1(
                        velStage[offset], posStage[offset],
                        vaVel, vbVel, vaPos, vbPos,
                        preJacVel[offset], preJacPos[offset],
                        effVel[offset], effPos[offset]);
                    break;
                case 2:
                    updateConstraint2(
                        &velStage[offset], &posStage[offset],
                        vaVel, vbVel, vaPos, vbPos,
                        &preJacVel[offset], &preJacPos[offset],
                        &effVel[offset], &effPos[offset]);
                    break;
                case 3:
                    updateConstraint3(
                        &velStage[offset], &posStage[offset],
                        vaVel, vbVel, vaPos, vbPos,
                        &preJacVel[offset], &preJacPos[offset],
                        &effVel[offset], &effPos[offset],
                        false);
                    break;
                }
                offset += d;
            }

            // collision constraints, always dim 3 with friction cone
            for (size_t c = pureCount; c < constraintCount; c++) {
                int iA = pairs[c].first;
                int iB = pairs[c].second;

                updateConstraint3(
                    &velStage[offset], &posStage[offset], 
                    virDVel[iA], virDVel[iB], 
                    virDPos[iA], virDPos[iB],
                    &preJacVel[offset], &preJacPos[offset],
                    &effVel[offset], &effPos[offset],
                    true
                );
                offset += 3;
            }
        }
    }

} // namespace OWSolver
