#include "OWWorld.hpp"
#include <map>
#include <cmath>

static uint64_t g_nextUID = 1;

OWWorld* OWWorld::Active = nullptr;

OWWorld::OWWorld() {
    Active = this;
    worldBody.setUID(0);
    worldBody.setStatic(true);
}

OWWorld::~OWWorld() {
    if (Active == this) Active = nullptr;
}

OWSolver::Body* OWWorld::createDynamicBox(const glm::vec3& pos, const glm::vec3& size, float mass) {
    auto b = std::make_unique<OWSolver::Body>(g_nextUID++);
    b->setShape(OWSolver::Body::Shape_Box);
    b->setSize(size);
    b->setBoxMassAndInertia(mass, size);
    b->setWorldCFrame(OWSolver::CoordinateFrame(glm::mat3(1.0f), pos));
    b->setFriction(0.3f);
    b->setRestitution(0.0f);
    b->setCanCollide(true);
    OWSolver::Body* raw = b.get();
    bodies.push_back(std::move(b));
    cw.addBody(raw);
    return raw;
}

OWSolver::Body* OWWorld::createStaticBox(const glm::vec3& pos, const glm::vec3& size) {
    auto b = std::make_unique<OWSolver::Body>(g_nextUID++);
    b->setStatic(true);
    b->setShape(OWSolver::Body::Shape_Box);
    b->setSize(size);
    b->setWorldCFrame(OWSolver::CoordinateFrame(glm::mat3(1.0f), pos));
    b->setFriction(0.3f);
    b->setRestitution(0.0f);
    b->setCanCollide(true);
    OWSolver::Body* raw = b.get();
    bodies.push_back(std::move(b));
    cw.addBody(raw);
    return raw;
}

void OWWorld::updateBodyShape(OWSolver::Body* body) {
    cw.updateBodyShape(body);
}

void OWWorld::addBodyToCollision(OWSolver::Body* body) {
    cw.addBody(body);
}

void OWWorld::removeBodyFromCollision(OWSolver::Body* body) {
    cw.removeBody(body);
}

bool OWWorld::raycastClosest(const glm::vec3& start, const glm::vec3& end, RayHit& outHit, const std::vector<const OWSolver::Body*>& ignoreBodies) const {
    OWSolver::CollisionWorld::RayHit hit;
    if (!cw.raycastClosest(start, end, hit, ignoreBodies)) {
        return false;
    }
    outHit.point = hit.point;
    outHit.normal = hit.normal;
    outHit.body = hit.body;
    return true;
}

void OWWorld::addPersistentConstraint(OWSolver::Constraint* c) {
    persistentConstraints.push_back(c);
}

void OWWorld::removePersistentConstraint(OWSolver::Constraint* c) {
    auto& v = persistentConstraints;
    v.erase(std::remove(v.begin(), v.end(), c), v.end());
}

void OWWorld::step(float dt) {
    accumulator += dt;
    int steps = 0;
    while (accumulator >= FIXED_DT && steps < MAX_SUBSTEPS) {
        substep();
        accumulator -= FIXED_DT;
        steps++;
    }
    if (steps == MAX_SUBSTEPS) {
        accumulator = 0.0f;
    }
}

void OWWorld::substep() {
    constexpr float GRAVITY = 196.2f;
    for (auto& b : bodies) {
        if (b->isStatic()) continue;
        if (!b->isRoot()) continue;
        b->accumulateForce(glm::vec3(0, -GRAVITY * b->getBranchMass(), 0));
    }
    
    if (preSubstepCallback) preSubstepCallback();

    cw.syncTransforms();
    cw.detectCollisions();

    std::map<OWSolver::ContactManager::PairKey, std::vector<OWSolver::ContactPoint>, OWSolver::ContactManager::PairKeyCompare> freshContacts;
    std::map<OWSolver::ContactManager::PairKey, std::pair<OWSolver::Body*, OWSolver::Body*>, OWSolver::ContactManager::PairKeyCompare> bodyLookup;
    cw.extractContacts(freshContacts, bodyLookup);
    cm.update(freshContacts, bodyLookup);

    // build solver inputs uh 1 SimBodyInput per root body
    std::map<uint64_t, int> rootUidToIndex;
    std::vector<OWSolver::SimBodyInput> inputs;
    std::vector<OWSolver::Body*> solverBodies;

    for (size_t i = 0; i < bodies.size(); i++) {
        OWSolver::Body* b = bodies[i].get();
        if (!b->isRoot()) continue;
        rootUidToIndex[b->getUID()] = (int)inputs.size();
        solverBodies.push_back(b);

        OWSolver::SimBodyInput in{};
        OWSolver::CoordinateFrame cf = b->getWorldCFrame();
        in.position = cf.translation;
        in.orientation = cf.rotation;
        in.linearVelocity = b->getLinearVelocity();
        in.angularVelocity = b->getAngularVelocity();
        in.externalForce = b->getExternalForce();
        in.externalTorque = b->getExternalTorque();
        in.externalImpulse = b->getExternalImpulse();
        in.externalAngularImpulse = b->getExternalRotationalImpulse();

        if (b->isStatic()) {
            in.massInv = 0.0f;
            in.inertiaInv = glm::mat3(0.0f);
            in.effectiveMassMultiplier = 0.0f;
        } else {
            b->recomputeBranchProperties();
            in.massInv = 1.0f / b->getBranchMass();
            glm::mat3 Iworld = b->getBranchInertiaWorldAtPoint(b->getBranchCofmWorld());
            float det = glm::determinant(Iworld);
            if (std::abs(det) > 1e-12f) {
                in.inertiaInv = glm::inverse(Iworld);
            } else {
                in.inertiaInv = glm::mat3(0.0f);
            }
            in.effectiveMassMultiplier = 1.0f;
        }
        inputs.push_back(in);
    }

    rootUidToIndex[0] = (int)inputs.size();
    {
        OWSolver::SimBodyInput in{};
        in.massInv = 0.0f;
        in.inertiaInv = glm::mat3(0.0f);
        in.effectiveMassMultiplier = 0.0f;
        inputs.push_back(in);
        solverBodies.push_back(&worldBody);
    }

    // build constraint arrays
    std::vector<OWSolver::Constraint*> constraints;
    std::vector<OWSolver::BodyPairIndices> pairs;
    std::vector<uint8_t> dims;

    // persistent constraints (upright, drive, etc.) (go inside constraints)
    for (auto* c : persistentConstraints) {
        auto itA = rootUidToIndex.find(c->getBodyA()->getUID());
        auto itB = rootUidToIndex.find(c->getBodyB()->getUID());
        if (itA == rootUidToIndex.end() || itB == rootUidToIndex.end()) continue;
        constraints.push_back(c);
        pairs.push_back({itA->second, itB->second});
        dims.push_back(c->getDimension());
    }

    // collision constraints
    size_t collisionCount = cm.gatherConstraints(constraints, pairs, dims, rootUidToIndex);

    // solve
    std::vector<OWSolver::SimBodyOutput> outputs;
    solver.Solve(inputs, constraints, pairs, dims, collisionCount, FIXED_DT, outputs);

    // write outputs back to bodies (skip world body and static bodies)
    for (size_t i = 0; i < solverBodies.size(); i++) {
        OWSolver::Body* b = solverBodies[i];
        if (b == &worldBody) continue;
        if (b->isStatic()) continue;
        OWSolver::CoordinateFrame cf(outputs[i].orientation, outputs[i].position);
        b->setWorldCFrame(cf);
        b->setLinearVelocity(outputs[i].linearVelocity);
        b->setAngularVelocity(outputs[i].angularVelocity);
    }

    for (auto& b : bodies) {
        if (!b->isRoot()) continue;
        b->clearAccumulators();
    }
}

bool OWWorld::isBodyInContact(OWSolver::Body* body) const {
    return cm.isBodyInContact(body->getRoot()->getUID());
}

void OWWorld::setBodyContactResponse(OWSolver::Body* body, bool enabled) {
    cw.setBodyContactResponse(body, enabled);
}

void OWWorld::setBodyFriction(OWSolver::Body* body, float friction) {
    cw.setBodyFriction(body, friction);
}

std::vector<OWSolver::Body*> OWWorld::getBodiesInAABB(const glm::vec3& min, const glm::vec3& max, const std::vector<const OWSolver::Body*>& ignore) {
    return cw.getBodiesInAABB(min, max, ignore);
}

void OWWorld::updateBodyFriction(OWSolver::Body* body, float friction) {
    body->setFriction(friction);
    cw.setBodyFriction(body, friction);  // for new contacts
    cm.updateFrictionForBody(body->getRoot()->getUID(), friction);  // for existing contacts
}