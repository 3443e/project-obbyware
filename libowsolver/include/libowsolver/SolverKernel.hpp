#pragma once
#include "libowsolver/SolverConfig.hpp"
#include "libowsolver/ConstraintJacobian.hpp"
#include "libowsolver/ConstraintVariables.hpp"
#include <vector>
#include <cstdint>

namespace OWSolver {    
    // One EffectiveMassPair per Jacobian row
    // computes  W * J^t for each constraint
    void ComputeEffectiveMasses(std::vector<EffectiveMassPair>&  effectiveMassesVel, std::vector<EffectiveMassPair>& effectiveMassesPos, const std::vector<ConstraintJacobianPair>& jacobians, const std::vector<BodyPairIndices>& pairs, const std::vector<uint8_t>& dimensions, const std::vector<SolverBodyMassAndInertia>& massAndInertia, const SolverConfig& config);

    // yeh
    void ApplyEffectiveMassMultipliers(std::vector<EffectiveMassPair>& effectiveMassesVel, std::vector<EffectiveMassPair>& effectiveMassesPos, const std::vector<BodyPairIndices>& pairs, const std::vector<uint8_t>& dimensions, const std::vector<float>& multipliers, const SolverConfig& config);

    // precondition Jacobians and reaction vectors
    // computes P^-1 * J and P^-1 * R
    void PreconditionConstraintEquations(
        std::vector<ConstraintJacobianPair>& precondJacobiansVel,
        std::vector<ConstraintJacobianPair>& precondJacobiansPos,
        std::vector<ConstraintVariables>& velStage,
        std::vector<ConstraintVariables>& posStage,
        const std::vector<ConstraintJacobianPair>& jacobians,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        const std::vector<uint8_t>& useBlock,
        const std::vector<float>& sorVel,
        const std::vector<float>& sorPos,
        const std::vector<EffectiveMassPair>& effectiveMassesVel,
        const std::vector<EffectiveMassPair>& effectiveMassesPos,
        const SolverConfig& config
    );

    // initialize virtual displacements from cached impulses
    void InitVirtualDisplacements(
        VirtualDisplacementArray& virDVel,
        VirtualDisplacementArray& virDPos,
        const std::vector<ConstraintVariables>& velStage,
        const std::vector<ConstraintVariables>& posStage,
        const std::vector<EffectiveMassPair>& effectiveMassesVel,
        const std::vector<EffectiveMassPair>& effectiveMassesPos,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        const SolverConfig& config
    );

    // main PGS kernel loop
    // runs pgsIterations iterations over all constraints
    void SolveKernel(
        std::vector<ConstraintVariables>& velStage,
        std::vector<ConstraintVariables>& posStage,
        VirtualDisplacementArray& virDVel,
        VirtualDisplacementArray& virDPos,
        const std::vector<ConstraintJacobianPair>& precondJacobiansVel,
        const std::vector<ConstraintJacobianPair>& precondJacobiansPos,
        const std::vector<EffectiveMassPair>& effectiveMassesVel,
        const std::vector<EffectiveMassPair>& effectiveMassesPos,
        const std::vector<BodyPairIndices>& pairs,
        const std::vector<uint8_t>& dimensions,
        size_t collisionCount,
        const SolverConfig& config
    );

} // namespace OWSolver
