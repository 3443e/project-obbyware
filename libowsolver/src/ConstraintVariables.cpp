#include "libowsolver/ConstraintVariables.hpp"
#include "libowsolver/SolverConfig.hpp"
#include <cmath>
#include <algorithm>

namespace OWSolver {
    static float recomputeSor(float sor, float relError, const SolverConfig::ModulationParams& cfg) {
        float newSor = cfg.aggressiveValue;
        if (relError > cfg.thresholdMin) {
            float t = std::min(relError, cfg.thresholdMax);
            t = std::max(t, cfg.thresholdMin);
            t = (t - cfg.thresholdMin) / (cfg.thresholdMax - cfg.thresholdMin);
            newSor = t * cfg.conservativeValue + (1.0f - t) * cfg.aggressiveValue;
        }
        if (newSor > sor) {
            sor = cfg.easingUpToAggressive * newSor + (1.0f - cfg.easingUpToAggressive) * sor;
        } else {
            sor = cfg.easingDownToConservative * newSor + (1.0f - cfg.easingDownToConservative) * sor;
        }
        return sor;
    }

    static float calculateCacheDampingFactor(float fit, float oldFactor, const SolverConfig::ModulationParams& cfg) {
        float t = (std::max(std::min(fit, cfg.thresholdMax), cfg.thresholdMin) - cfg.thresholdMin) / (cfg.thresholdMax - cfg.thresholdMin);
        float factor = t * cfg.conservativeValue + (1.0f - t) * cfg.aggressiveValue;
        float easing = factor < oldFactor ? cfg.easingDownToConservative : cfg.easingUpToAggressive;
        return easing * factor + (1.0f - easing) * oldFactor;
    }

    void ConstraintCache::cache(const ConstraintVariables& velStage, const ConstraintVariables& posStage, float sorVel, float sorPos, bool isCollision, const SolverConfig& config) {
        float velFit = velocityImpulseRegression.testFitNextDataPointSecondOrder(velStage.impulse);
        float posFit = positionImpulseRegression.testFitNextDataPointSecondOrder(posStage.impulse);

        velocityCacheDamping = calculateCacheDampingFactor(velFit, velocityCacheDamping, config.cacheVStageModulation);
        positionCacheDamping = calculateCacheDampingFactor(posFit, positionCacheDamping, config.cachePStageModulation);

        if (!isCollision) {
            velocitySor = recomputeSor(sorVel, velFit, config.sorConstraintsModulation);
            positionSor = recomputeSor(sorPos, posFit, config.sorConstraintsModulation);
        } else {
            float vf = velocityImpulseRegression.testFitNextDataPointZeroOrder(velStage.impulse);
            float pf = positionImpulseRegression.testFitNextDataPointZeroOrder(posStage.impulse);
            velocitySor = recomputeSor(sorVel, vf, config.sorCollisionsModulation);
            positionSor = recomputeSor(sorPos, pf, config.sorCollisionsModulation);
        }

        velocityImpulse = velocityCacheDamping * velStage.impulse;
        velocityReaction = velStage.reaction;
        positionImpulse = positionCacheDamping * posStage.impulse;
        positionReaction = posStage.reaction;

        velocityImpulseRegression.addDataPoint(velocityImpulse, 1.0f);
        positionImpulseRegression.addDataPoint(positionImpulse, 1.0f);
    }

} // namespace OWSolver
