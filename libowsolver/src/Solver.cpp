#define GLM_ENABLE_EXPERIMENTAL
#include "libowsolver/Solver.hpp"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <cassert>

namespace OWSolver {

    Solver::Solver(const SolverConfig& config) : Config(config) {}

    void Solver::IntegrateVelocities(SolverBodyDynamicProperties& props, const SolverBodyMassAndInertia& mass, const SimBodyInput& input, float dt, const SolverConfig& config) {
        // linear: v_integrated = v + massInv * (F*dt + J)
        props.integratedLinearVelocity = input.linearVelocity + mass.massInvVelStage * (input.externalForce * dt + input.externalImpulse);

        // angular: w_integrated = exp(-damping*dt) * w + I_inv * (T*dt + L)
        float dampFactor = std::exp(-config.angularDamping * dt);
        props.integratedAngularVelocity = dampFactor * input.angularVelocity + input.inertiaInv * (input.externalTorque * dt + input.externalAngularImpulse);
    }

    void Solver::IntegratePositions( SolverBodyDynamicProperties& props, const VirtualDisplacement& velDelta, const VirtualDisplacement& posDelta, float dt) {
        // Apply constraint impulses to get final velocity
        props.linearVelocity = props.integratedLinearVelocity + velDelta.lin;
        props.angularVelocity = props.integratedAngularVelocity + velDelta.ang;

        // Apply positional correction
        props.position += posDelta.lin;

        // Apply rotational correction
        if (glm::length(posDelta.ang) > 1e-10f) {
            float angle = glm::length(posDelta.ang);
            glm::vec3 axis = posDelta.ang / angle;
            glm::mat3 rot = glm::mat3_cast(glm::angleAxis(angle, axis));
            props.orientation = rot * props.orientation;
        }

        // Integrate positions
        props.position += props.linearVelocity * dt;

        // Integrate orientation
        if (glm::length(props.angularVelocity) > 1e-10f) {
            float angle = glm::length(props.angularVelocity) * dt;
            glm::vec3 axis = glm::normalize(props.angularVelocity);
            glm::mat3 rot = glm::mat3_cast(glm::angleAxis(angle, axis));
            props.orientation = rot * props.orientation;
        }

        // Orthonormalize to prevent drift
        glm::vec3 c0 = glm::normalize(props.orientation[0]);
        glm::vec3 c2 = glm::normalize(glm::cross(c0, props.orientation[1]));
        glm::vec3 c1 = glm::cross(c2, c0);
        props.orientation = glm::mat3(c0, c1, c2);
    }

    // goes through 11 steps
    void Solver::Solve(
        const std::vector<SimBodyInput>& bodies,
        std::vector<Constraint*>& constraints,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        size_t collisionCount,
        float dt,
        std::vector<SimBodyOutput>& outputs
    ) {
        size_t bodyCount = bodies.size();
        size_t constraintCount = constraints.size();

        // Total DOFs across all constraints
        size_t totalDim = 0;
        for (uint8_t d : dimensions) {
            totalDim += d;
        }

        // --- 1. Build mass and inertia arrays ---
        std::vector<SolverBodyMassAndInertia> massAndInertia(bodyCount);
        for (size_t i = 0; i < bodyCount; i++) {
            const SimBodyInput& b = bodies[i];
            glm::mat3 ii = b.inertiaInv;
            massAndInertia[i].massInvVelStage = b.massInv;
            massAndInertia[i].posToVelMassRatio = b.massInv > 0.0f ? std::pow(b.massInv, Config.stabilizationMassReductionPower) / b.massInv : 0.0f;
            massAndInertia[i].inertiaDiagonal = glm::vec3(ii[0][0], ii[1][1], ii[2][2]);
            massAndInertia[i].inertiaOffDiagonal = glm::vec3(ii[0][1], ii[0][2], ii[1][2]);
        }

        // --- 2. Integrate velocities, init dynamic props ---
        std::vector<SolverBodyDynamicProperties> bodyProps(bodyCount);
        for (size_t i = 0; i < bodyCount; i++) {
            const SimBodyInput& b = bodies[i];
            bodyProps[i].position = b.position;
            bodyProps[i].orientation = b.orientation;
            bodyProps[i].linearVelocity = b.linearVelocity;
            bodyProps[i].angularVelocity = b.angularVelocity;
            IntegrateVelocities(bodyProps[i], massAndInertia[i], b, dt, Config);
        }

        // --- 3. Allocate constraint arrays ---
        std::vector<ConstraintJacobianPair> jacobians(totalDim);
        std::vector<ConstraintVariables> velStage(totalDim);
        std::vector<ConstraintVariables> posStage(totalDim);
        std::vector<float> sorVel(totalDim, 1.0f);
        std::vector<float> sorPos(totalDim, 1.0f);
        std::vector<uint8_t> useBlock(totalDim, Config.blockPGSEnabled);

        // --- 4. Build constraint equations ---
        size_t offset = 0;
        for (size_t c = 0; c < constraintCount; c++) {
            uint8_t d = dimensions[c];
            int iA = pairs[c].first;
            int iB = pairs[c].second;

            // static body gets zero dynamic props
            SolverBodyDynamicProperties emptyProps{};

            const SolverBodyDynamicProperties& bA = (iA >= 0 && iA < (int)bodyCount) ? bodyProps[iA] : emptyProps;
            const SolverBodyDynamicProperties& bB = (iB >= 0 && iB < (int)bodyCount) ? bodyProps[iB] : emptyProps;

            constraints[c]->restoreCacheAndBuildEquation(
                &jacobians[offset],
                &velStage[offset],
                &posStage[offset],
                &sorVel[offset],
                &sorPos[offset],
                &useBlock[offset],
                bA, bB, Config, dt
            );
            offset += d;
        }

        // --- 5. Compute effective masses ---
        std::vector<EffectiveMassPair> effVel(totalDim);
        std::vector<EffectiveMassPair> effPos(totalDim);
        ComputeEffectiveMasses(effVel, effPos, jacobians, pairs, dimensions, massAndInertia, Config);

        // --- 6. Precondition ---
        std::vector<ConstraintJacobianPair> preJacVel(totalDim);
        std::vector<ConstraintJacobianPair> preJacPos(totalDim);
        PreconditionConstraintEquations(preJacVel, preJacPos, velStage, posStage, jacobians, pairs, dimensions, useBlock, sorVel, sorPos, effVel, effPos, Config);

        // --- 7. Apply effective mass multipliers (sleeping bodies) ---
        std::vector<float> multipliers(bodyCount);
        for (size_t i = 0; i < bodyCount; i++) {
            multipliers[i] = bodies[i].effectiveMassMultiplier;
        }
        ApplyEffectiveMassMultipliers(effVel, effPos, pairs, dimensions, multipliers, Config);

        // --- 8. Init virtual displacements ---
        VirtualDisplacementArray virDVel(bodyCount);
        VirtualDisplacementArray virDPos(bodyCount);
        virDVel.reset();
        virDPos.reset();
        InitVirtualDisplacements(virDVel, virDPos, velStage, posStage, effVel, effPos, pairs, dimensions, Config);

        // --- 9. Run PGS kernel ---
        SolveKernel(velStage, posStage, virDVel, virDPos, preJacVel, preJacPos, effVel, effPos, pairs, dimensions, collisionCount, Config);

        // --- 10. Cache constraint results ---
        offset = 0;
        for (size_t c = 0; c < constraintCount; c++) {
            uint8_t d = dimensions[c];
            constraints[c]->updateBrokenState( &velStage[offset], &posStage[offset], Config);
            constraints[c]->cache(&velStage[offset], &posStage[offset], &sorVel[offset], &sorPos[offset], Config);
            offset += d;
        }

        // --- 11. Integrate positions and write outputs ---
        outputs.resize(bodyCount);
        for (size_t i = 0; i < bodyCount; i++) {
            IntegratePositions(bodyProps[i], virDVel[i], virDPos[i], dt);
            outputs[i].position = bodyProps[i].position;
            outputs[i].orientation = bodyProps[i].orientation;
            outputs[i].linearVelocity = bodyProps[i].linearVelocity;
            outputs[i].angularVelocity = bodyProps[i].angularVelocity;
        }
    }

} // namespace OWSolver
