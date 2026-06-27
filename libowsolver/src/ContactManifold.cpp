// FILE: src/ContactManifold.cpp
#include "libowsolver/ContactManifold.hpp"
#include <algorithm>

namespace OWSolver {

    static uint64_t globalConstraintUID = 0;

    ContactManifold::ContactManifold(Body* a, Body* b)
        : rootA(a), rootB(b), referenceCount(1) {}

    ContactManifold::~ContactManifold() {
        for (ConstraintCollision* c : collisions) {
            delete c;
        }
        collisions.clear();
    }

    bool ContactManager::isGrounded(uint64_t rootUid, float upThreshold) const {
        for (const auto& kv : manifolds) {
            ContactManifold* m = kv.second;
            const auto& collisions = m->getCollisions();
            if (collisions.empty()) continue;

            bool isA = (m->getRootA()->getUID() == rootUid);
            bool isB = (m->getRootB()->getUID() == rootUid);
            if (!isA && !isB) continue;

            for (ConstraintCollision* c : collisions) {
                glm::vec3 n = c->getNormal();

                float ny = isA ? -n.y : n.y;
                if (ny > upThreshold) return true;
            }
        }
        return false;
    }
    void ContactManifold::update(const std::vector<ContactPoint>& fresh) {

        std::vector<const ContactPoint*> clean;
        clean.reserve(fresh.size());
        for (const ContactPoint& cp : fresh) {
            if (glm::dot(cp.normal, cp.normal) > 0.8f) {  // length squared > 0.8 ~ normal is sane
                clean.push_back(&cp);
            }
        }

        if (clean.empty()) {
            for (ConstraintCollision* c : collisions) {
                delete c;
            }
            collisions.clear();
            return;
        }

        size_t newIdx = 0;
        size_t cachedIdx = 0;
        for (; cachedIdx < collisions.size() && newIdx < clean.size(); cachedIdx++, newIdx++) {
            const ContactPoint& cp = *clean[newIdx];
            ConstraintCollision* c = collisions[cachedIdx];
            c->setNormal(cp.normal);
            c->setPointA(cp.positionOnA);
            c->setDepth(cp.depth);
            c->setFriction(cp.friction);
            c->setRestitution(cp.restitution);
        }

        for (; newIdx < clean.size(); newIdx++) {
            const ContactPoint& cp = *clean[newIdx];
            ConstraintCollision* c = new ConstraintCollision(rootA, rootB);
            globalConstraintUID++;
            c->setUID(globalConstraintUID);
            c->setNormal(cp.normal);
            c->setPointA(cp.positionOnA);
            c->setDepth(cp.depth);
            c->setFriction(cp.friction);
            c->setRestitution(cp.restitution);
            collisions.push_back(c);
        }

        // Remove vanished collisions
        if (clean.size() < collisions.size()) {
            for (size_t i = clean.size(); i < collisions.size(); i++) {
                delete collisions[i];
            }
            collisions.resize(clean.size());
        }
    }

    // ---- ContactManager ----

    void ContactManager::update(const std::map<PairKey, std::vector<ContactPoint>, PairKeyCompare>& freshByPair, const std::map<PairKey, std::pair<Body*, Body*>, PairKeyCompare>& bodyLookup) {
        for (auto it = manifolds.begin(); it != manifolds.end(); ) {
            const PairKey& key = it->first;
            ContactManifold* manifold = it->second;

            auto freshIt = freshByPair.find(key);
            if (freshIt != freshByPair.end()) {
                manifold->update(freshIt->second);
                ++it;
            } else {
                std::vector<ContactPoint> empty;
                manifold->update(empty);
                if (manifold->getCollisions().empty()) {
                    delete manifold;
                    it = manifolds.erase(it);
                } else {
                    ++it;
                }
            }
        }

        for (const auto& kv : freshByPair) {
            const PairKey& key = kv.first;
            if (manifolds.find(key) == manifolds.end()) {
                auto it = bodyLookup.find(key);
                if (it == bodyLookup.end()) continue;
                Body* a = it->second.first;
                Body* b = it->second.second;
                ContactManifold* m = new ContactManifold(a, b);
                m->update(kv.second);
                manifolds[key] = m;
            }
        }
    }

    bool ContactManager::isBodyInContact(uint64_t rootUid) const {
        for (const auto& kv : manifolds) {
            ContactManifold* m = kv.second;
            const auto& collisions = m->getCollisions();
            if (collisions.empty()) continue;
            if (m->getRootA()->getUID() == rootUid || m->getRootB()->getUID() == rootUid) {
                return true;
            }
        }
        return false;
    }
    size_t ContactManager::gatherConstraints(std::vector<Constraint*>& outConstraints, std::vector<BodyPairIndices>& outPairs, std::vector<uint8_t>& outDims, const std::map<uint64_t, int>& rootUidToSolverIndex) const {
        size_t added = 0;
        for (const auto& kv : manifolds) {
            ContactManifold* m = kv.second;
            const auto& collisions = m->getCollisions();
            if (collisions.empty()) continue;

            auto itA = rootUidToSolverIndex.find(m->getRootA()->getUID());
            auto itB = rootUidToSolverIndex.find(m->getRootB()->getUID());
            if (itA == rootUidToSolverIndex.end() || itB == rootUidToSolverIndex.end()) {
                continue;
            }
            int idxA = itA->second;
            int idxB = itB->second;

            for (ConstraintCollision* c : collisions) {
                outConstraints.push_back(c);
                outPairs.push_back(BodyPairIndices(idxA, idxB));
                outDims.push_back(3);  // collisions are always 3-DOF (normal + 2 tangents)
                added++;
            }
        }
        return added;
    }

    void ContactManager::clear() {
        for (auto& kv : manifolds) {
            delete kv.second;
        }
        manifolds.clear();
    }

    ContactManifold* ContactManager::findManifold(uint64_t uidA, uint64_t uidB) const {
        PairKey key = (uidA < uidB) ? PairKey(uidA, uidB) : PairKey(uidB, uidA);
        auto it = manifolds.find(key);
        return it != manifolds.end() ? it->second : nullptr;
    }

    size_t ContactManager::totalCollisionCount() const {
        size_t total = 0;
        for (const auto& kv : manifolds) {
            total += kv.second->getCollisions().size();
        }
        return total;
    }

    void ContactManager::updateFrictionForBody(uint64_t rootUid, float newFriction) {
        for (auto& kv : manifolds) {
            ContactManifold* m = kv.second;
            const auto& collisions = m->getCollisions();
            
            bool isA = (m->getRootA()->getUID() == rootUid);
            bool isB = (m->getRootB()->getUID() == rootUid);
            if (!isA && !isB) continue;
            
            // get the other body's friction
            float otherFriction = 0.0f;
            if (isA) {
                otherFriction = m->getRootB()->getFriction();
            } else {
                otherFriction = m->getRootA()->getFriction();
            }
            
            float c0 = std::clamp(newFriction, 0.0f, 2.0f);
            float c1 = std::clamp(otherFriction, 0.0f, 2.0f);
            float combinedFriction;
            if ((c0 <= 1.0f && c1 <= 1.0f) || (c0 >= 1.0f && c1 >= 1.0f)) {
                combinedFriction = std::min(c0, c1);
            } else {
                combinedFriction = c0 + c1 - 1.0f;
            }
            
            for (ConstraintCollision* c : collisions) {
                c->setFriction(combinedFriction);
            }
        }
    }

} // namespace OWSolver
