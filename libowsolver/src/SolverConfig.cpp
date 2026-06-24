#include "libowsolver/SolverConfig.hpp"
#include <cmath>
#include <boost/math/constants/constants.hpp>

namespace OWSolver {
    SolverConfig::SolverConfig(Type type) {
        pgsIterations = 20;

        collisionRestitutionThreshold              = 1.0f;
        collisionPenetrationMargin                 = 0.05f;
        collisionPenetrationMarginMax              = 0.05f;    // PGSPenetrationMarginMax / 1000000
        collisionPenetrationMarginMin              = 0.0001f;  // PGSPenetrationMarginMin / 1000000
        collisionPenetrationVelocityForMinMargin   = 20.0f;
        collisionPenetrationMarginMaxBumpProportions = 0.05f;  // PGSPenetrationMarginMaxBump / 100
        collisionPenetrationResolutionDamping      = 0.7f;     // PGSPenetrationResolutionDamping / 10
        collisionFrictionStaticToDynamicThreshold  = 1.0f;
        collisionFrictionStaticScale               = 1.0f;
        collisionFrictionDynamicScale              = 1.0f;

        align2AxesFrictionConstant                 = 0.02f;
        align2AxesPositionStageFrictionConstant    = 0.01f;
        align2AxesMaxCorrectiveAngle               = 3.0f;
        align2AxesCorrectionDamping                = 1.0f;     // PGSAlign2AxesCorrectionDamping / 10

        ballInSocketMaxCorrectionByStabilization   = 0.3f;
        ballInSocketCorrectionDamping              = 1.0f;     // PGSBallInSocketCorrectionDamping / 10

        sorConstraintsModulation = {1.0f, 0.01f, 1.5f, 0.75f, 0.0001f, 0.2f};
        sorCollisionsModulation  = {0.05f, 0.01f, 1.9f, 1.0f, 0.0001f, 0.2f};

        stabilizationMassReductionPower = 0.3f;
        stabilizationInertiaScale       = 0.05f;

        cacheVStageModulation = {0.5f, 2.0f,  0.93f, 0.99f, 0.01f, 0.0002f};
        cachePStageModulation = {0.5f, 1.5f,  0.70f, 0.99f, 0.01f, 0.0002f};

        velCacheDamping         = 1.0f;
        posCacheDamping         = 1.0f;
        constraintCachingEnabled = true;

        angularDamping          = 0.911328f; // -ln(0.99621) / (1/240)
        updateSimBodies         = true;
        integrateOnlyPositions  = false;

        blockPGSEnabled         = true;
        virtualMassesEnabled    = true;
        velocityStageSOREnabled = true;
        positionStageSOREnabled = true;
        useSimIslands           = false;

        inconsistentConstraintDetectorEnabled           = false;
        inconsistentConstraintMaxIterations             = 0;
        inconsistentConstraintBallInSocketResidualThreshold = 0.02f;
        inconsistentConstraintAlign2AxesThreshold       = 0.003f;
        inconsistentConstraintCollisionThreshold        = 0.02f;
        inconsistentConstraintCollisionBaseThreshold    = 0.001f;
        inconsistentConstraintDeltaThreshold            = 0.0001f;
        printConvergenceDiagnostics                     = false;

        if (type == Type_InconsistencyDetector) {
            inconsistentConstraintDetectorEnabled  = true;
            inconsistentConstraintMaxIterations    = 100000;
            pgsIterations                          = 100;
            posCacheDamping                        = 0.0f;
            velCacheDamping                        = 0.0f;
            constraintCachingEnabled               = false;
            blockPGSEnabled                        = false;
            velocityStageSOREnabled                = false;
            positionStageSOREnabled                = false;
            collisionFrictionStaticScale           = 0.0f;
            collisionFrictionDynamicScale          = 0.0f;
            updateSimBodies                        = false;
            stabilizationMassReductionPower        = 0.1f;
            useSimIslands                          = true;
        }

        if (type == Type_PositionalCorrection) {
            posCacheDamping         = 0.0f;
            velCacheDamping         = 0.0f;
            constraintCachingEnabled = false;
            blockPGSEnabled         = false;
            velocityStageSOREnabled = false;
            positionStageSOREnabled = false;
            collisionFrictionStaticScale  = 0.0f;
            collisionFrictionDynamicScale = 0.0f;
            integrateOnlyPositions  = true;
        }
    }
} // namespace OWSolver
