// produces ContactPoint lists keyed by (rootA, rootB) for ContactManager.

#pragma once
#include "libowsolver/Body.hpp"
#include "libowsolver/ContactManifold.hpp"
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <glm/glm.hpp>
#include <map>
#include <vector>

// bullet types are forward-declared in the GLOBAL namespace.
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btDbvtBroadphase;
class btCollisionWorld;
class btCollisionObject;
class btBoxShape;
class btSphereShape;

namespace OWSolver {

    class CollisionWorld {
    public:
        CollisionWorld();
        ~CollisionWorld();

        // Register a Body for collision detection. The body must outlive its registration. Shape is derived from Body::getShape() and Body::getSize().
        void addBody(Body* body);
        void removeBody(Body* body);

        // set whether a body generates contact response (constraint forces) false = "ghost" mode (detects overlaps but doesn't block).
        void setBodyContactResponse(Body* body, bool enabled);

        void setBodyFriction(Body* body, float friction);
        // Call this BEFORE detectCollisions()
        void syncTransforms();

        // Ray hit result
        struct RayHit {
            glm::vec3 point;
            glm::vec3 normal;
            Body* body;
        };

        bool raycastClosest(const glm::vec3& start, const glm::vec3& end, RayHit& outHit, const std::vector<const Body*>& ignoreBodies = {}) const;

        void detectCollisions();
        void updateBodyShape(Body* body);

        void extractContacts(
            std::map<ContactManager::PairKey, std::vector<ContactPoint>, ContactManager::PairKeyCompare>& outContacts,
            std::map<ContactManager::PairKey, std::pair<Body*, Body*>, ContactManager::PairKeyCompare>& outBodyLookup
        );

        std::vector<Body*> getBodiesInAABB(const glm::vec3& min, const glm::vec3& max, const std::vector<const Body*>& ignoreBodies);

        void BeginBatchLoad();
        void EndBatchLoad();
    private:

        btDefaultCollisionConfiguration* config = nullptr;
        btCollisionDispatcher* dispatcher = nullptr;
        btDbvtBroadphase* broadphase = nullptr;
        btCollisionWorld* world = nullptr;

        struct BodyEntry {
            btCollisionObject* obj = nullptr;
            btBoxShape* boxShape = nullptr;
            btSphereShape* sphereShape = nullptr;
            btCylinderShape* cylShape = nullptr;
            btConvexHullShape* convexShape = nullptr;
        };
        std::map<Body*, BodyEntry> entries;
    };

} // namespace OWSolver
