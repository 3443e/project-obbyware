#pragma once
#include "libowsolver/SolverConfig.hpp"
#include "libowsolver/ConstraintJacobian.hpp"
#include "libowsolver/ConstraintVariables.hpp"
#include <glm/glm.hpp>
#include <cstdint>
#include <limits>

namespace OWSolver {

    class Body;

    class Constraint {
    public:
        enum Types {
            Types_Collision,
            Types_Align2Axes,
            Types_BallInSocket,
            Types_AngularVelocity,
            Types_LinearVelocity,
            Types_AchievePosition,
            Types_BodyAngularVelocity,
            Types_LinearSpring,
            Types_LegacyBreakableBallInSocket,
            Types_LegacyAngularVelocity,
            Types_Count
        };

        enum Convergence {
            Convergence_Converges,
            Convergence_Diverges,
            Convergence_Undetermined
        };

        virtual ~Constraint();

        // called by solver every frame
        // this restores cached impulses then builds Jacobian + reaction vectors
        void restoreCacheAndBuildEquation(ConstraintJacobianPair* jacobian, ConstraintVariables* velStage, ConstraintVariables* posStage, float* sorVel, float* sorPos, uint8_t* useBlock, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt);

        // called by solver after kernel to write impulses back to cache
        void cache(const ConstraintVariables* velStage, const ConstraintVariables* posStage, const float* sorVel, const float* sorPos, const SolverConfig& config);

        // called by solver to check if constraint broke this frame
        void updateBrokenState(const ConstraintVariables* velStage, const ConstraintVariables* posStage, const SolverConfig& config);

        // check for physics analyzer or something
        virtual Convergence testPGSConvergence(const float* disp, const float* residuals, const float* deltaResiduals, const SolverConfig& config) { return Convergence_Converges; }

        uint8_t getDimension() const {
            return dimensions;
        }

        bool isBroken() const {
            return broken;
        }
        Types getType() const {
            return type;
        }

        Body* getBodyA() {
            return bodyA;
        }

        Body* getBodyB() {
            return bodyB;
        }

        const Body* getBodyA() const {
            return bodyA;
        }

        const Body* getBodyB() const {
            return bodyB;
        }

        void setUID(uint64_t id) {
            uid = id;
        }

        bool hasValidUID() const {
            return uid != 0;
        }
        uint64_t getUID() const {
            return uid;
        }

    protected:
        Constraint(Types type, Body* bodyA, Body* bodyB, uint8_t dimensions);

        ConstraintCache& getCache(unsigned d) {
            return cacheData[d];
        }

        const ConstraintCache& getCache(unsigned d) const {
            return cacheData[d];
        }

    private:
        // inherited classes implement this to fill in Jacobian and reaction vectors
        virtual void buildEquation(ConstraintJacobianPair* jacobian,uint8_t* useBlock, ConstraintVariables* velStage, ConstraintVariables* posStage, const SolverBodyDynamicProperties& bodyA, const SolverBodyDynamicProperties& bodyB, const SolverConfig& config, float dt) = 0;

        virtual bool computeBrokenState( const ConstraintVariables* velStage,const ConstraintVariables* posStage,const SolverConfig& config) const { return false; }

        // no copy
        Constraint(const Constraint&) = delete;
        Constraint& operator=(const Constraint&) = delete;

        uint8_t dimensions;
        Types type;
        bool broken = false;
        Body* bodyA;
        Body* bodyB;
        uint64_t uid = 0;

        ConstraintCache* cacheData = nullptr;
    };

} // namespace OWSolver
