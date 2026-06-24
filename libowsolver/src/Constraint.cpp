#include "libowsolver/Constraint.hpp"
#include <limits>

namespace OWSolver {
    Constraint::Constraint(Types t, Body* a, Body* b, uint8_t dims) : type(t), bodyA(a), bodyB(b), dimensions(dims) {
        cacheData = new ConstraintCache[dimensions];
    }

    Constraint::~Constraint() {
        delete[] cacheData;
    }

    // called by solver every frame
    void Constraint::restoreCacheAndBuildEquation(ConstraintJacobianPair* jacobian,
        ConstraintVariables* velStage,
        ConstraintVariables* posStage,
        float* sorVel, float* sorPos,
        uint8_t* useBlock,
        const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB,
        const SolverConfig& config,
        float dt)
    {
        for (unsigned i = 0; i < dimensions; i++) {
            velStage[i].minImpulseValue = -std::numeric_limits<float>::infinity();
            velStage[i].maxImpulseValue = std::numeric_limits<float>::infinity();
            posStage[i].minImpulseValue = -std::numeric_limits<float>::infinity();
            posStage[i].maxImpulseValue = std::numeric_limits<float>::infinity();

            jacobian[i].reset();

            // default to block PGS unless constraint overrides
            useBlock[i] = config.blockPGSEnabled;

            // restore cached impulses and SOR values
            getCache(i).readCache(velStage[i], posStage[i], sorVel[i], sorPos[i]);

            // apply cache damping
            velStage[i].impulse *= config.velCacheDamping;
            posStage[i].impulse *= config.posCacheDamping;
        }

        buildEquation(jacobian, useBlock, velStage, posStage, bodyA, bodyB, config, dt);
    }

    void Constraint::cache( const ConstraintVariables* velStage,const ConstraintVariables* posStage, const float* sorVel, const float* sorPos,const SolverConfig& config) {
        for (unsigned i = 0; i < dimensions; i++) {
            cacheData[i].cache(velStage[i], posStage[i], sorVel[i], sorPos[i], (type == Types_Collision), config);
        }
    }

    void Constraint::updateBrokenState(const ConstraintVariables* velStage, const ConstraintVariables* posStage, const SolverConfig& config) {
        if (!broken) {
            broken = computeBrokenState(velStage, posStage, config);
        }
    }

} // namespace OWSolver
