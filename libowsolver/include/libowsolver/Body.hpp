//
// Design (matching the original):
// - Each Body has parent/root pointers forming a tree.
// - Only the ROOT body owns simulated state (position/orientation/velocity).
//   Leaf bodies follow the root rigidly via stored local transforms.
// - When a contact is detected on a leaf (e.g. the Torso), it is mapped
//   to its root (HumanoidRootPart) before being fed to the solver. The
//   contact point's world position stays on the leaf surface, so the
//   lever arm is correct, but the mass/inertia used are the assembly's
//   aggregated values.
// - Aggregated mass + inertia about the assembly COM are cached on the
//   root and recomputed when the tree changes (weld/unweld/mass edit).
#pragma once
#include "libowsolver/CoordinateFrame.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <cstdint>
#include <string>

namespace OWSolver {

    // Forward-declared so Body can friend us
    class ContactManager;
    class CollisionWorld;

    class Body {
    public:
        enum Shape {
            Shape_None,
            Shape_Box,
            Shape_Sphere,
            Shape_Cylinder,      // Roblox Cylinder (aligned along X axis)
            Shape_Wedge,         // Roblox WedgePart (triangular prism)
            Shape_CornerWedge,   // Roblox CornerWedgePart (corner ramp)
        };

        explicit Body(uint64_t uid = 0);
        ~Body();

        // -------- Identity --------
        uint64_t getUID() const { return uid; }
        void setUID(uint64_t u) { uid = u; }

        // -------- Assembly tree --------
        // Weld a child to this body. The child becomes a leaf; its root
        // pointer is updated to our root. localCFrame is the child's
        // transform relative to THIS body's local frame.
        void weldChild(Body* child, const CoordinateFrame& localCFrame);

        // Detach this body from its parent (becomes its own root).
        void detach();

        Body* getParent() const { return parent; }
        Body* getRoot();
        const Body* getRoot() const;
        bool isRoot() const { return parent == nullptr; }

        const std::vector<Body*>& getChildren() const { return children; }

        // -------- Physical properties (per-body, in body frame) --------
        void setMass(float m);
        float getMass() const { return mass; }

        // Sets a box inertia tensor for a box of given full size and current mass.
        void setBoxMassAndInertia(float massValue, const glm::vec3& size);

        // Sets a sphere inertia tensor for a sphere of given radius.
        void setSphereMassAndInertia(float massValue, float radius);

        // Manual override.
        void setInertiaBody(const glm::mat3& I) { inertiaBody = I; markBranchDirty(); }
        glm::mat3 getInertiaBody() const { return inertiaBody; }

        // shape (for collision)
        void setShape(Shape s) { shape = s; }
        Shape getShape() const { return shape; }

        void setSize(const glm::vec3& s) { size = s; }
        glm::vec3 getSize() const { return size; }

        void setCanCollide(bool c) { canCollide = c; }
        bool getCanCollide() const { return canCollide; }

        // static / anchored 
        void setStatic(bool s);
        bool isStatic() const { return staticBody; }

        // for root: returns the stored world transform.
        // for leaf: walks up to root and combines local transforms.
        CoordinateFrame getWorldCFrame() const;
        glm::vec3 getWorldPosition() const { return getWorldCFrame().translation; }
        glm::mat3 getWorldOrientation() const { return getWorldCFrame().rotation; }

        // Only valid on a root. sets the body's world transform directly.
        void setWorldCFrame(const CoordinateFrame& cf);

        // velocity (root only)
        glm::vec3 getLinearVelocity() const;
        glm::vec3 getAngularVelocity() const;
        void setLinearVelocity(const glm::vec3& v);
        void setAngularVelocity(const glm::vec3& v);


        glm::vec3 getVelocityAtPoint(const glm::vec3& worldPoint) const;

        // external accumulators (root only)
        // Forces/torques are cleared each frame before the solver runs.
        void accumulateForce(const glm::vec3& f);
        void accumulateTorque(const glm::vec3& t);
        void accumulateForceAtPoint(const glm::vec3& f, const glm::vec3& worldPoint);
        void accumulateImpulse(const glm::vec3& i);
        void accumulateImpulseAtPoint(const glm::vec3& i, const glm::vec3& worldPoint);
        void accumulateRotationalImpulse(const glm::vec3& ri);
        void clearAccumulators();

        glm::vec3 getExternalForce() const { return externalForce; }
        glm::vec3 getExternalTorque() const { return externalTorque; }
        glm::vec3 getExternalImpulse() const { return externalImpulse; }
        glm::vec3 getExternalRotationalImpulse() const { return externalRotationalImpulse; }

        // (root only, cached) --------
        float getBranchMass() const { return branchMass; }
        glm::vec3 getBranchCofmWorld() const;
        glm::mat3 getBranchInertiaWorldAtPoint(const glm::vec3& worldPoint) const;

        // mark this body's branch as needing recomputation.
        void markBranchDirty();
        // recompute and cache aggregated properties. only necessary on root.
        void recomputeBranchProperties();

        // overrides
        void setFriction(float f) { friction = f; }
        float getFriction() const { return friction; }
        void setRestitution(float r) { restitution = r; }
        float getRestitution() const { return restitution; }

        bool isTruss = false;
        void setTruss(bool t) { isTruss = t; }
        bool getIsTruss() const { return isTruss; }

        void setName(const std::string& n) { name = n; }
        const std::string& getName() const { return name; }

        glm::vec3 getBranchIBodyV3() const;

        void setLocalCFrame(const CoordinateFrame& cf) {
            localCFrame = cf;
            getRoot()->markBranchDirty();
        }

    private:
        uint64_t uid;
        std::string name;

        Body* parent = nullptr;
        Body* rootCache = nullptr;
        CoordinateFrame localCFrame;
        std::vector<Body*> children;

        float mass = 1.0f;
        glm::mat3 inertiaBody = glm::mat3(1.0f);
        glm::vec3 size = glm::vec3(1.0f);
        Shape shape = Shape_Box;
        bool canCollide = true;
        bool staticBody = false;
        float friction = 0.3f;
        float restitution = 0.0f;

        CoordinateFrame worldCFrame;
        glm::vec3 linearVelocity{0.0f};
        glm::vec3 angularVelocity{0.0f};

        // external accumulators, only meaningful on root
        glm::vec3 externalForce{0.0f};
        glm::vec3 externalTorque{0.0f};
        glm::vec3 externalImpulse{0.0f};
        glm::vec3 externalRotationalImpulse{0.0f};

        bool branchDirty = true;
        float branchMass = 0.0f;
        glm::vec3 branchCofmOffsetLocal{0.0f};

        glm::mat3 branchInertiaLocal = glm::mat3(0.0f);

        void recomputeBranchRecursive(const CoordinateFrame& rootCFrame, float& outMass, glm::vec3& outCofmWorld, glm::mat3& outInertiaWorld) const;

        void refreshRootCache(Body* newRoot);
    };

    // Helpers
    inline Body* Body::getRoot() {
        if (rootCache) return rootCache;
        Body* r = this;
        while (r->parent) r = r->parent;
        return r;
    }

    inline const Body* Body::getRoot() const {
        if (rootCache) return rootCache;
        const Body* r = this;
        while (r->parent) r = r->parent;
        return r;
    }

} // namespace OWSolver
