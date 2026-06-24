#pragma once
#include "libowsolver/SolverConfig.hpp"
#include "libowsolver/SolverBody.hpp"
#include "libowsolver/ConstraintJacobian.hpp"
#include "libowsolver/ConstraintVariables.hpp"
#include "libowsolver/Constraint.hpp"
#include "libowsolver/SolverKernel.hpp"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace OWSolver {
    // a simulated body which is given into the solver each "frame"
    struct SimBodyInput {
        glm::vec3 position;
        glm::mat3 orientation;
        glm::vec3 linearVelocity;
        glm::vec3 angularVelocity;

        // accumulated forces and impulses for this frame
        glm::vec3 externalForce;
        glm::vec3 externalTorque;
        glm::vec3 externalImpulse;
        glm::vec3 externalAngularImpulse;

        glm::mat3 inertiaInv{0.0f};

        float massInv = 0.0f;  // default: static (anchored)

        // 1.0 = fully simulated, 0.0 = static/sleeping (collapses mass to inf)
        float effectiveMassMultiplier = 1.0f;
    };

    // results written back after solve
    struct SimBodyOutput {
        glm::vec3 position;
        glm::mat3 orientation;
        glm::vec3 linearVelocity;
        glm::vec3 angularVelocity;
    };

    // one solve call: takes bodies + constraints, returns updated body states
    class Solver {
    public:
        explicit Solver(const SolverConfig& config = SolverConfig());

        // Main entry point
        // bodies:      list of simulated bodies
        // constraints: list of constraints between bodies
        // pairs:       which body indices each constraint connects
        // dimensions:  DOF count per constraint
        // collisionCount: how many of the constraints are collisions (must be last)
        // dt:          timestep
        // outputs:     written with new positions/velocities
        void Solve(
            const std::vector<SimBodyInput>& bodies,
            std::vector<Constraint*>& constraints,
            const std::vector<BodyPairIndices>& pairs,
            const std::vector<uint8_t>& dimensions,
            size_t collisionCount,
            float dt,
            std::vector<SimBodyOutput>& outputs
        );

        void SetConfig(const SolverConfig& config) {
            Config = config;
        }

        const SolverConfig& GetConfig() const {
            return Config;
        }

    private:
        SolverConfig Config;

        // integrate velocities using external forces
        // Returns integratedLinearVelocity and integratedAngularVelocity
        static void IntegrateVelocities(SolverBodyDynamicProperties& props, const SolverBodyMassAndInertia& mass, const SimBodyInput& input, float dt, const SolverConfig& config);

        // integrate positions using solved velocities + virtual displacements
        static void IntegratePositions(SolverBodyDynamicProperties& props, const VirtualDisplacement& velDelta, const VirtualDisplacement& posDelta, float dt);
    };

} // namespace OWSolver
