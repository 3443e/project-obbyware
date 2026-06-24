// addContactManifold / removeContactManifold machinery from Solver.cpp.
//
// Design (matching the original):
// - A manifold is keyed by sorted (rootUidA, rootUidB), NOT leaf UIDs.
//   This is critical: a contact detected on the Torso uses the HumanoidRootPart's
//   UID, so the manifold persists across frames even if Bullet's contact point
//   happens to be reported on a different leaf of the same assembly.
// - Each manifold holds up to N ConstraintCollision objects. Per frame we
//   match new contacts to existing constraints BY INDEX (exactly like the
//   original does at line 2944 of Solver.cpp). Existing constraints keep
//   their cached impulse -> warm-starting. New constraints are appended;
//   vanished ones are deleted.
// - Reference counting lets multiple subsystems hold a manifold without
//   premature deletion.
#pragma once
#include "libowsolver/ConstraintTypes.hpp"
#include "libowsolver/Body.hpp"
#include <vector>
#include <map>
#include <cstdint>

namespace OWSolver {

    // A single fresh contact point detected by the collision system this frame.
    struct ContactPoint {
        glm::vec3 positionOnA;  // world-space contact point on A's surface
        glm::vec3 normal;        // unit vector pointing from A to B
        float depth;             // negative = penetrating (matches original convention)
        float friction;          // combined friction coefficient
        float restitution;       // combined restitution coefficient
    };

    // One manifold per body-pair. Holds 0..N ConstraintCollision objects.
    class ContactManifold {
    public:
        ContactManifold(Body* rootA, Body* rootB);
        ~ContactManifold();

        Body* getRootA() const { return rootA; }
        Body* getRootB() const { return rootB; }

        // Update with fresh contacts from this frame's collision detection.
        // Existing ConstraintCollision objects are reused by index (warm-start).
        // New contacts get fresh ConstraintCollision; vanished ones are deleted.
        void update(const std::vector<ContactPoint>& freshContacts);
        // The currently-active collision constraints for this pair.
        const std::vector<ConstraintCollision*>& getCollisions() const { return collisions; }

        int referenceCount = 0;

    private:
        Body* rootA;
        Body* rootB;
        std::vector<ConstraintCollision*> collisions;
        uint64_t nextUID = 1;  // for assigning constraint UIDs
    };

    // Owns all manifolds. Keyed by sorted pair of root UIDs.
    class ContactManager {
    public:
        // Update all manifolds for this frame.
        // - For each (rootA, rootB) pair in freshContacts, lookup-or-create the
        //   manifold, then call manifold.update().
        // - For each manifold that had NO contacts this frame, leave it
        //   alive but empty
        //
        // freshContactsByPair: keyed by sorted (rootA, rootB) UIDs.
        using PairKey = std::pair<uint64_t, uint64_t>;  // (smaller, larger) UID
        struct PairKeyCompare {
            bool operator()(const PairKey& a, const PairKey& b) const {
                if (a.first != b.first) return a.first < b.first;
                return a.second < b.second;
            }
        };

        void update(const std::map<PairKey, std::vector<ContactPoint>, PairKeyCompare>& freshContactsByPair,
                    const std::map<PairKey, std::pair<Body*, Body*>, PairKeyCompare>& bodyLookup);

        // Gather all active collision constraints from all manifolds.
        // Appends to outConstraints, outPairs, outDims. Returns the count
        // of collision constraints added.
        size_t gatherConstraints(
            std::vector<Constraint*>& outConstraints,
            std::vector<BodyPairIndices>& outPairs,
            std::vector<uint8_t>& outDims,
            const std::map<uint64_t, int>& rootUidToSolverIndex
        ) const;

        // Clear all manifolds (for shutdown / world reset).
        void clear();

        // Lookup a manifold by pair (returns nullptr if not found).
        ContactManifold* findManifold(uint64_t uidA, uint64_t uidB) const;
        bool isGrounded(uint64_t rootUid, float upThreshold = 0.7f) const;
                // Returns true if the body with the given root UID has any active contact.
        bool isBodyInContact(uint64_t rootUid) const;
        // Total active collision count across all manifolds.
        size_t totalCollisionCount() const;

                // Update friction on all contacts for this body
        void updateFrictionForBody(uint64_t rootUid, float newFriction);
    private:
        std::map<PairKey, ContactManifold*, PairKeyCompare> manifolds;
    };

} // namespace OWSolver
