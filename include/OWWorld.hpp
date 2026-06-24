#pragma once
#include "libowsolver/Body.hpp"
#include "libowsolver/CollisionWorld.hpp"
#include "libowsolver/ContactManifold.hpp"
#include "libowsolver/Solver.hpp"
#include "libowsolver/Constraint.hpp"
#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include <functional>

class OWWorld {
public:
    static OWWorld* Active;

    OWWorld();
    ~OWWorld();

    OWSolver::Body* createDynamicBox(const glm::vec3& pos, const glm::vec3& size, float mass);
    OWSolver::Body* createStaticBox(const glm::vec3& pos, const glm::vec3& size);


    void setBodyContactResponse(OWSolver::Body* body, bool enabled);
    void setBodyFriction(OWSolver::Body* body, float friction);
    void updateBodyFriction(OWSolver::Body* body, float friction);

    bool isBodyInContact(OWSolver::Body* body) const;
    void step(float dt);

    // collision shape management (called by OWPart::SetSize)
    void updateBodyShape(OWSolver::Body* body);
    void addBodyToCollision(OWSolver::Body* body);
    void removeBodyFromCollision(OWSolver::Body* body);

    // persistent constraints (added to every substep's solve).
    // is used sed by OWPlayerController for the upright constraint.
    void addPersistentConstraint(OWSolver::Constraint* c);
    void removePersistentConstraint(OWSolver::Constraint* c);

    OWSolver::Body* getWorldBody() {
        return &worldBody;
    }

    size_t getContactCount() const { return cm.totalCollisionCount(); }

    // direct contact manager access for freaky stuff
    const OWSolver::ContactManager& getContactManager() const {
        return cm;
    }

    struct RayHit {
        glm::vec3 point;
        glm::vec3 normal;
        OWSolver::Body* body;
    };

    // THIS SKIPS IGNORED BODIES
    bool raycastClosest(const glm::vec3& start, const glm::vec3& end, RayHit& outHit, const std::vector<const OWSolver::Body*>& ignoreBodies = {}) const;

    /*
    so like this called every 240Hz substep, AFTER gravity
    is applied but BEFORE the solver runs. use this to apply forces that
    need to be active every substep (like the hip controller).
    */
    using PreSubstepCallback = std::function<void()>;
    void setPreSubstepCallback(PreSubstepCallback cb) { preSubstepCallback = std::move(cb); }
    std::vector<OWSolver::Body*> getBodiesInAABB(const glm::vec3& min, const glm::vec3& max, const std::vector<const OWSolver::Body*>& ignore);
private:
    PreSubstepCallback preSubstepCallback;
    OWSolver::CollisionWorld cw;
    OWSolver::ContactManager cm;
    OWSolver::Solver solver;
    std::vector<std::unique_ptr<OWSolver::Body>> bodies;

    OWSolver::Body worldBody;

    std::vector<OWSolver::Constraint*> persistentConstraints;

    float accumulator = 0.0f;
    static constexpr float FIXED_DT = 1.0f / 240.0f;
    static constexpr int MAX_SUBSTEPS = 8;

    void substep();
};