#pragma once

namespace OWSolver {
    struct SolverConfig {
        enum Type {
            Type_Default,
            Type_InconsistencyDetector,
            Type_PositionalCorrection,
        };

        explicit SolverConfig(Type type = Type_Default);

        // kernel
        unsigned pgsIterations;

        // Collisions
        float collisionRestitutionThreshold;
        float collisionPenetrationMargin;
        float collisionPenetrationMarginMax;
        float collisionPenetrationMarginMin;
        float collisionPenetrationMarginMaxBumpProportions;
        float collisionPenetrationResolutionDamping;
        float collisionPenetrationVelocityForMinMargin;
        float collisionFrictionStaticToDynamicThreshold;
        float collisionFrictionStaticScale;
        float collisionFrictionDynamicScale;

        // Align2Axes
        float align2AxesFrictionConstant;
        float align2AxesPositionStageFrictionConstant;
        float align2AxesMaxCorrectiveAngle;
        float align2AxesCorrectionDamping;

        // BallInSocket
        float ballInSocketMaxCorrectionByStabilization;
        float ballInSocketCorrectionDamping;

        // SOR modulation
        struct ModulationParams {
            float thresholdMax;
            float thresholdMin;
            float aggressiveValue;
            float conservativeValue;
            float easingUpToAggressive;
            float easingDownToConservative;
        };
        
        ModulationParams sorConstraintsModulation;
        ModulationParams sorCollisionsModulation;
        ModulationParams cacheVStageModulation;
        ModulationParams cachePStageModulation;

        // stabilization
        float stabilizationMassReductionPower;
        float stabilizationInertiaScale;

        // Cache
        float velCacheDamping;
        float posCacheDamping;
        bool constraintCachingEnabled;

        // Integration
        float angularDamping;
        bool updateSimBodies;
        bool integrateOnlyPositions;

        // Solver options
        bool blockPGSEnabled;
        float velocityStageSOREnabled;
        float positionStageSOREnabled;
        bool virtualMassesEnabled;
        bool useSimIslands;

        // Inconsistency detector
        bool inconsistentConstraintDetectorEnabled;
        unsigned inconsistentConstraintMaxIterations;
        float inconsistentConstraintBallInSocketResidualThreshold;
        float inconsistentConstraintDeltaThreshold;
        float inconsistentConstraintAlign2AxesThreshold;
        float inconsistentConstraintCollisionThreshold;
        float inconsistentConstraintCollisionBaseThreshold;
        bool printConvergenceDiagnostics;
    };
} // namespace OWSolver
