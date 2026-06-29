#include "libowsolver/CollisionWorld.hpp"

#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <LinearMath/btTransform.h>
#include <LinearMath/btVector3.h>
#include <btBulletCollisionCommon.h>
#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

constexpr float bulletCollisionMargin = 0.045f;

namespace OWSolver {

    static btVector3 glm2bt(const glm::vec3& v) {
        return btVector3(v.x, v.y, v.z);
    }
    static glm::vec3 bt2glm(const btVector3& v) {
        return glm::vec3(v.x(), v.y(), v.z());
    }
    static btTransform mat3ToBtTransform(const glm::mat3& R, const glm::vec3& t) {
        btMatrix3x3 basis(
            R[0][0], R[1][0], R[2][0],
            R[0][1], R[1][1], R[2][1],
            R[0][2], R[1][2], R[2][2]
        );
        btTransform tr;
        tr.setBasis(basis);
        tr.setOrigin(glm2bt(t));
        return tr;
    }

    // ray callback that skips specific collision objects
    class FilteredRayCallback : public btCollisionWorld::ClosestRayResultCallback {
    public:
        FilteredRayCallback(const btVector3& from, const btVector3& to, const std::set<const btCollisionObject*>& ignoreSet) : btCollisionWorld::ClosestRayResultCallback(from, to), m_ignoreSet(ignoreSet) {}

        const std::set<const btCollisionObject*>& m_ignoreSet;

        virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) override {
            // tell Bullet to keep looking by returning 1.0
            if (m_ignoreSet.count(rayResult.m_collisionObject) > 0) {
                return 1.0f;
            }
            return btCollisionWorld::ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
        }
    };

    // -------- CollisionWorld --------

    CollisionWorld::CollisionWorld() {
        config = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(config);
        broadphase = new btDbvtBroadphase();
        world = new btCollisionWorld(dispatcher, broadphase, config);
    }

    CollisionWorld::~CollisionWorld() {
        for (auto& kv : entries) {
            BodyEntry& e = kv.second;
            if (e.obj) {
                world->removeCollisionObject(e.obj);
                delete e.obj;
            }
            if (e.boxShape) delete e.boxShape;
            if (e.sphereShape) delete e.sphereShape;
        }
        entries.clear();

        delete world;
        delete broadphase;
        delete dispatcher;
        delete config;
    }

    void CollisionWorld::addBody(Body* body) {
        if (entries.find(body) != entries.end()) return;
        if (!body->getCanCollide()) return;
        if (body->getShape() == Body::Shape_None) return;

        BodyEntry e{};
        e.obj = new btCollisionObject();
        e.boxShape = nullptr;
        e.sphereShape = nullptr;
        e.cylShape = nullptr;
        e.convexShape = nullptr;
        switch (body->getShape()) {
            case Body::Shape_Box: {
                glm::vec3 halfExtents = body->getSize() * 0.5f;
                e.boxShape = new btBoxShape(glm2bt(halfExtents));
                e.boxShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.boxShape);
                break;
            }
            case Body::Shape_Sphere: {
                float radius = body->getSize().x * 0.5f;
                e.sphereShape = new btSphereShape(radius);
                e.obj->setCollisionShape(e.sphereShape);
                break;
            }
            case Body::Shape_Cylinder: {
                glm::vec3 halfExtents = body->getSize() * 0.5f;
                e.cylShape = new btCylinderShapeX(btVector3(halfExtents.x, halfExtents.y, halfExtents.z));
                e.cylShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.cylShape);
                break;
            }
            case Body::Shape_Wedge: {
                e.convexShape = new btConvexHullShape();
                float x = (body->getSize().x * 0.5f) - bulletCollisionMargin;
                float y = (body->getSize().y * 0.5f) - bulletCollisionMargin;
                float z = (body->getSize().z * 0.5f) - bulletCollisionMargin;
                
                e.convexShape->addPoint(btVector3( x, y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, z), false);
                e.convexShape->addPoint(btVector3(-x, y, -z), false);
                e.convexShape->addPoint(btVector3(-x, -y, -z), false);
                e.convexShape->addPoint(btVector3(-x, -y, z), true);
                e.convexShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.convexShape);
                break;
            }
            case Body::Shape_CornerWedge: {
                e.convexShape = new btConvexHullShape();
                float x = (body->getSize().x * 0.5f) - bulletCollisionMargin;
                float y = (body->getSize().y * 0.5f) - bulletCollisionMargin;
                float z = (body->getSize().z * 0.5f) - bulletCollisionMargin;
                
                e.convexShape->addPoint(btVector3(-x, -y, -z), false);
                e.convexShape->addPoint(btVector3(-x, -y, z), false);
                e.convexShape->addPoint(btVector3( x, y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, z), true);
                e.convexShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.convexShape);
                break;
            }
            default:
                e.obj->setCollisionShape(nullptr);
                break;
        }

        if (body->isStatic()) {
            e.obj->setCollisionFlags(e.obj->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
        } else {
            e.obj->setCollisionFlags(e.obj->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        }

        e.obj->setUserPointer(body);
        
        world->addCollisionObject(e.obj);
        entries[body] = e;
    }

    void CollisionWorld::removeBody(Body* body) {
        auto it = entries.find(body);
        if (it == entries.end()) return;
        BodyEntry& e = it->second;
        if (e.obj) {
            world->removeCollisionObject(e.obj);
            delete e.obj;
        }
        if (e.boxShape) { delete e.boxShape; e.boxShape = nullptr; }
        if (e.sphereShape) { delete e.sphereShape; e.sphereShape = nullptr; }
        if (e.cylShape) { delete e.cylShape; e.cylShape = nullptr; }
        if (e.convexShape) { delete e.convexShape; e.convexShape = nullptr; }
        entries.erase(it);
    }

        void CollisionWorld::updateBodyShape(Body* body) {
        auto it = entries.find(body);
        if (it == entries.end()) return;
        BodyEntry& e = it->second;

        world->removeCollisionObject(e.obj);

        if (e.boxShape) { delete e.boxShape; e.boxShape = nullptr; }
        if (e.sphereShape) { delete e.sphereShape; e.sphereShape = nullptr; }
        if (e.cylShape) { delete e.cylShape; e.cylShape = nullptr; }
        if (e.convexShape) { delete e.convexShape; e.convexShape = nullptr; }

        switch (body->getShape()) {
            case Body::Shape_Box: {
                glm::vec3 halfExtents = body->getSize() * 0.5f;
                e.boxShape = new btBoxShape(glm2bt(halfExtents));
                e.boxShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.boxShape);
                break;
            }
            case Body::Shape_Sphere: {
                float radius = body->getSize().x * 0.5f;
                e.sphereShape = new btSphereShape(radius);
                // Spheres do not need a collision margin in Roblox
                e.obj->setCollisionShape(e.sphereShape);
                break;
            }
            case Body::Shape_Cylinder: {
                glm::vec3 halfExtents = body->getSize() * 0.5f;
                e.cylShape = new btCylinderShapeX(btVector3(halfExtents.x, halfExtents.y, halfExtents.z));
                e.cylShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.cylShape);
                break;
            }
            case Body::Shape_Wedge: {
                e.convexShape = new btConvexHullShape();
                float x = (body->getSize().x * 0.5f) - bulletCollisionMargin;
                float y = (body->getSize().y * 0.5f) - bulletCollisionMargin;
                float z = (body->getSize().z * 0.5f) - bulletCollisionMargin;
                
                e.convexShape->addPoint(btVector3( x,  y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, z), false);
                e.convexShape->addPoint(btVector3(-x, y, -z), false);
                e.convexShape->addPoint(btVector3(-x, -y, -z), false);
                e.convexShape->addPoint(btVector3(-x, -y, z), true);
                e.convexShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.convexShape);
                break;
            }
            case Body::Shape_CornerWedge: {
                e.convexShape = new btConvexHullShape();
                float x = (body->getSize().x * 0.5f) - bulletCollisionMargin;
                float y = (body->getSize().y * 0.5f) - bulletCollisionMargin;
                float z = (body->getSize().z * 0.5f) - bulletCollisionMargin;
                
                e.convexShape->addPoint(btVector3(-x, -y, -z), false);
                e.convexShape->addPoint(btVector3(-x, -y, z), false);
                e.convexShape->addPoint(btVector3( x, y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, -z), false);
                e.convexShape->addPoint(btVector3( x, -y, z), true);
                e.convexShape->setMargin(bulletCollisionMargin);
                e.obj->setCollisionShape(e.convexShape);
                break;
            }
            default:
                e.obj->setCollisionShape(nullptr);
                break;
        }

        world->addCollisionObject(e.obj);
    }

    void CollisionWorld::syncTransforms() {
        for (auto& kv : entries) {
            Body* body = kv.first;
            const BodyEntry& e = kv.second;
            if (!e.obj || !e.obj->getCollisionShape()) continue;
            CoordinateFrame cf = body->getWorldCFrame();
            btTransform tr = mat3ToBtTransform(cf.rotation, cf.translation);
            e.obj->setWorldTransform(tr);
            e.obj->setInterpolationWorldTransform(tr);
        }
    }

    void CollisionWorld::detectCollisions() {
        world->performDiscreteCollisionDetection();
    }

    void CollisionWorld::extractContacts(
        std::map<ContactManager::PairKey, std::vector<ContactPoint>, ContactManager::PairKeyCompare>& outContacts,
        std::map<ContactManager::PairKey, std::pair<Body*, Body*>, ContactManager::PairKeyCompare>& outBodyLookup)
    {
        outContacts.clear();
        outBodyLookup.clear();

        int numManifolds = dispatcher->getNumManifolds();
        for (int i = 0; i < numManifolds; i++) {
            btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);
            if (!manifold) continue;

            const btCollisionObject* obA = static_cast<const btCollisionObject*>(manifold->getBody0());
            const btCollisionObject* obB = static_cast<const btCollisionObject*>(manifold->getBody1());
            if (!obA || !obB) continue;

            Body* leafA = static_cast<Body*>(obA->getUserPointer());
            Body* leafB = static_cast<Body*>(obB->getUserPointer());
            if (!leafA || !leafB) continue;
        
            Body* rootA = leafA->getRoot();
            Body* rootB = leafB->getRoot();

            if (rootA == rootB) continue;
            if (rootA->isStatic() && rootB->isStatic()) continue;

            uint64_t uidA = rootA->getUID();
            uint64_t uidB = rootB->getUID();
            ContactManager::PairKey key = (uidA < uidB) ? std::make_pair(uidA, uidB) : std::make_pair(uidB, uidA);
            Body* keyFirst  = (uidA < uidB) ? rootA : rootB;
            Body* keySecond = (uidA < uidB) ? rootB : rootA;

            outBodyLookup[key] = std::make_pair(keyFirst, keySecond);

            int numContacts = manifold->getNumContacts();
            if (numContacts == 0) continue;

            std::vector<ContactPoint>& points = outContacts[key];
            points.reserve(numContacts);

            float c0 = std::clamp(leafA->getFriction(), 0.0f, 2.0f);
            float c1 = std::clamp(leafB->getFriction(), 0.0f, 2.0f);
            float combinedFriction;
            if ((c0 <= 1.0f && c1 <= 1.0f) || (c0 >= 1.0f && c1 >= 1.0f)) {
                combinedFriction = std::min(c0, c1);
            } else {
                combinedFriction = c0 + c1 - 1.0f;
            }
            float combinedRestitution = 0.5f * (leafA->getRestitution() + leafB->getRestitution());

            bool swapAB = (uidA > uidB);

            for (int c = 0; c < numContacts; c++) {
                const btManifoldPoint& mp = manifold->getContactPoint(c);

                float depth = mp.getDistance();
                if (depth > 0.0f) continue;

                btVector3 posA = mp.getPositionWorldOnA();
                btVector3 normalB = mp.m_normalWorldOnB; 

                ContactPoint cp;
                if (swapAB) {
                    cp.normal = bt2glm(normalB);
                    cp.positionOnA = bt2glm(mp.getPositionWorldOnB());
                } else {
                    cp.normal = -bt2glm(normalB);
                    cp.positionOnA = bt2glm(posA);
                }

                float nlen = std::sqrt(glm::dot(cp.normal, cp.normal));
                if (nlen < 0.9f) continue;
                cp.normal /= nlen;

                cp.depth = depth;
                cp.friction = combinedFriction;
                cp.restitution = combinedRestitution;

                points.push_back(cp);
            }

            if (points.empty()) {
                outContacts.erase(key);
            }
        }
    }

    // raycast
    bool CollisionWorld::raycastClosest(const glm::vec3& start, const glm::vec3& end, RayHit& outHit, const std::vector<const Body*>& ignoreBodies) const {
        std::set<const btCollisionObject*> ignoreSet;
        for (const Body* b : ignoreBodies) {
            auto it = entries.find(const_cast<Body*>(b));
            if (it != entries.end() && it->second.obj) {
                ignoreSet.insert(it->second.obj);
            }
        }

        btVector3 from = glm2bt(start);
        btVector3 to = glm2bt(end);
        FilteredRayCallback callback(from, to, ignoreSet);
        world->rayTest(from, to, callback);

        if (!callback.hasHit()) return false;

        outHit.point = bt2glm(callback.m_hitPointWorld);
        outHit.normal = bt2glm(callback.m_hitNormalWorld);
        outHit.body = static_cast<Body*>(callback.m_collisionObject->getUserPointer());
        return true;
    }

    void CollisionWorld::setBodyContactResponse(Body* body, bool enabled) {
        auto it = entries.find(body);
        if (it == entries.end()) return;
        btCollisionObject* obj = it->second.obj;
        if (!obj) return;
        if (enabled) {
            // clear the no-contact-response flag
            obj->setCollisionFlags(obj->getCollisionFlags() & ~btCollisionObject::CF_NO_CONTACT_RESPONSE);
        } else {
            // set the no-contact-response flag (ghost mode)
            obj->setCollisionFlags(obj->getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        }
    }

    void CollisionWorld::setBodyFriction(Body* body, float friction) {
        auto it = entries.find(body);
        if (it == entries.end()) return;
        btCollisionObject* obj = it->second.obj;
        if (!obj) return;
        obj->setFriction(friction);
        obj->setRollingFriction(0.0f);
    }

    std::vector<Body*> CollisionWorld::getBodiesInAABB(const glm::vec3& min, const glm::vec3& max, const std::vector<const Body*>& ignoreBodies) {
        std::vector<Body*> result;
    
        std::set<const Body*> ignoreSet(ignoreBodies.begin(), ignoreBodies.end());
        
        for (auto& kv : entries) {
            Body* body = kv.first;
            const auto& e = kv.second;
            if (!e.obj || !e.obj->getCollisionShape()) continue;
            if (ignoreSet.count(body) > 0) continue;
            
            btVector3 aabbMin, aabbMax;
            e.obj->getCollisionShape()->getAabb(e.obj->getWorldTransform(), aabbMin, aabbMax);
            
            // check overlap
            if (aabbMin.x() <= max.x && aabbMax.x() >= min.x &&
                aabbMin.y() <= max.y && aabbMax.y() >= min.y &&
                aabbMin.z() <= max.z && aabbMax.z() >= min.z) {
                result.push_back(body);
            }
        }
        return result;
    }

    void CollisionWorld::BeginBatchLoad() {
        if (world) {
            // tell Bullet to stop updating its broadphase pairs while we add bodies
            world->getPairCache()->setOverlapFilterCallback(nullptr);
        }
    }

    void CollisionWorld::EndBatchLoad() {
        if (world) {
            // re-evaluate all pairs now that everything is loaded
            world->getBroadphase()->resetPool(dispatcher);
            // restore default callback (null is default for btDbvtBroadphase, but we explicitly set it to be safe)
            world->getPairCache()->setOverlapFilterCallback(nullptr);
        }
    }
} // namespace OWSolver

    