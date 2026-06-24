#pragma once
#include <glm/glm.hpp>
#include <limits>

namespace OWSolver {
    // direct port of ConstraintVariables from solver/constraint.h
    struct ConstraintVariables {
        float minImpulseValue;
        float maxImpulseValue;
        float reaction;   /* desired velocity correction */
        float impulse;    // current accumulated impulse (cached between frames)

        ConstraintVariables()
            : minImpulseValue(-std::numeric_limits<float>::infinity())
            , maxImpulseValue(std::numeric_limits<float>::infinity())
            , reaction(0.0f)
            , impulse(0.0f)
        {}

        // helper functions ahead:

        // matching what Constraint.cpp uses
        static void setReaction(ConstraintVariables* vars, const glm::vec3& r) {
            vars[0].reaction = r.x;
            vars[1].reaction = r.y;
            vars[2].reaction = r.z;
        }

        static void setReaction(ConstraintVariables* vars, float x, float y) {
            vars[0].reaction = x;
            vars[1].reaction = y;
        }

        static void setImpulse(ConstraintVariables* vars, const glm::vec3& i) {
            vars[0].impulse = i.x;
            vars[1].impulse = i.y;
            vars[2].impulse = i.z;
        }

        static void setMinImpulses(ConstraintVariables* vars, const glm::vec3& mn) {
            vars[0].minImpulseValue = mn.x;
            vars[1].minImpulseValue = mn.y;
            vars[2].minImpulseValue = mn.z;
        }

        static void setMinImpulses(ConstraintVariables* vars, float v) {
            vars[0].minImpulseValue = v;
            vars[1].minImpulseValue = v;
            vars[2].minImpulseValue = v;
        }

        static void setMaxImpulses(ConstraintVariables* vars, const glm::vec3& mx) {
            vars[0].maxImpulseValue = mx.x;
            vars[1].maxImpulseValue = mx.y;
            vars[2].maxImpulseValue = mx.z;
        }

        static void setMaxImpulses(ConstraintVariables* vars, float v) {
            vars[0].maxImpulseValue = v;
            vars[1].maxImpulseValue = v;
            vars[2].maxImpulseValue = v;
        }
    };

    // idk what this is 
    struct MovingRegression {
        float confidence = 0.0f;
        float lastPoint= 0.0f;
        float lastTangent = 0.0f;
        float lastCurvature = 0.0f;

        float testFitNextDataPointSecondOrder(float y) const {
            float predicted = lastPoint + lastTangent + lastCurvature;
            float denom = std::max(std::abs(y), std::abs(predicted)) + 0.00001f;
            return confidence * std::abs(y - predicted) / denom;
        }

        float testFitNextDataPointZeroOrder(float y) const {
            float denom = std::max(std::abs(y), std::abs(lastPoint)) + 0.00001f;
            return confidence * std::abs(y - lastPoint) / denom;
        }

        void addDataPoint(float y, float weight) {
            float newTangent = y - lastPoint;
            float newCurvature = newTangent - lastTangent;
            lastCurvature = newCurvature;
            lastTangent = newTangent;
            lastPoint = y;
            confidence += 0.1f * (1.0f - confidence);
        }
    };

    // used to warm-start the solver and adapt SOR values
    struct ConstraintCache {
        float velocityImpulse = 0.0f;
        float velocityReaction = 0.0f;
        float velocitySor = 1.9f;
        float velocityCacheDamping = 1.0f;
        float positionImpulse = 0.0f;
        float positionReaction = 0.0f;
        float positionSor = 1.9f;
        float positionCacheDamping = 1.0f;

        MovingRegression velocityImpulseRegression;
        MovingRegression positionImpulseRegression;

        void readCache(ConstraintVariables& velStage, ConstraintVariables& posStage, float& sorVel, float& sorPos) const {
            velStage.impulse = velocityImpulse;
            velStage.reaction = velocityReaction;
            sorVel = velocitySor;
            posStage.impulse = positionImpulse;
            posStage.reaction = positionReaction;
            sorPos = positionSor;
        }

        void cache(const ConstraintVariables& velStage, const ConstraintVariables& posStage, float sorVel, float sorPos, bool isCollision, const struct SolverConfig& config);
    };

} // namespace OWSolver
