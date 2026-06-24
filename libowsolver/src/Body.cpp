// FILE: src/Body.cpp
#include "libowsolver/Body.hpp"
#include <algorithm>
#include <cmath>

namespace OWSolver {

    Body::Body(uint64_t uid_) : uid(uid_) {
        rootCache = this;
    }

    Body::~Body() {
        // detach from parent first
        if (parent) {
            auto& siblings = parent->children;
            siblings.erase(std::remove(siblings.begin(), siblings.end(), this), siblings.end());
            parent->markBranchDirty();
        }
        // children become roots of their own trees
        for (Body* c : children) {
            c->parent = nullptr;
            c->refreshRootCache(c);
            c->markBranchDirty();
        }
    }

    void Body::refreshRootCache(Body* newRoot) {
        rootCache = newRoot;
        for (Body* c : children) {
            c->refreshRootCache(newRoot);
        }
    }

    void Body::weldChild(Body* child, const CoordinateFrame& localCf) {
        // if child already had a parent, detach first
        if (child->parent) {
            child->detach();
        }
        child->parent = this;
        child->localCFrame = localCf;
        children.push_back(child);

        // propagate root identity down through the child's subtree
        Body* r = getRoot();
        child->refreshRootCache(r);

        // mark our root as needing recomputation
        r->markBranchDirty();
    }

    void Body::detach() {
        if (!parent) return;
        auto& siblings = parent->children;
        siblings.erase(std::remove(siblings.begin(), siblings.end(), this), siblings.end());
        parent->getRoot()->markBranchDirty();
        parent = nullptr;
        refreshRootCache(this);
        markBranchDirty();
    }

    void Body::setMass(float m) {
        mass = m;
        getRoot()->markBranchDirty();
    }

    void Body::setBoxMassAndInertia(float massValue, const glm::vec3& size_) {
        mass = massValue;
        size = size_;
        shape = Shape_Box;
        glm::vec3 sq = size_ * size_;
        // I = (m/12) * (other_dim_sq_sums)
        inertiaBody = glm::mat3(0.0f);
        inertiaBody[0][0] = (massValue / 12.0f) * (sq.y + sq.z);
        inertiaBody[1][1] = (massValue / 12.0f) * (sq.x + sq.z);
        inertiaBody[2][2] = (massValue / 12.0f) * (sq.x + sq.y);
        getRoot()->markBranchDirty();
    }

    void Body::setSphereMassAndInertia(float massValue, float radius) {
        mass = massValue;
        size = glm::vec3(radius * 2.0f);
        shape = Shape_Sphere;
        // I = (2/5) * m * r^2 (uniform sphere)
        float I = 0.4f * massValue * radius * radius;
        inertiaBody = glm::mat3(0.0f);
        inertiaBody[0][0] = I;
        inertiaBody[1][1] = I;
        inertiaBody[2][2] = I;
        getRoot()->markBranchDirty();
    }

    void Body::setStatic(bool s) {
        staticBody = s;
        getRoot()->markBranchDirty();
    }

    CoordinateFrame Body::getWorldCFrame() const {
        if (!parent) return worldCFrame;
        return parent->getWorldCFrame() * localCFrame;
    }

    void Body::setWorldCFrame(const CoordinateFrame& cf) {
        if (parent) return;  // silent no-op for safety
        worldCFrame = cf;
    }

    glm::vec3 Body::getLinearVelocity() const {
        return getRoot()->linearVelocity;
    }

    glm::vec3 Body::getAngularVelocity() const {
        return getRoot()->angularVelocity;
    }

    void Body::setLinearVelocity(const glm::vec3& v) {
        Body* r = getRoot();
        r->linearVelocity = v;
    }

    void Body::setAngularVelocity(const glm::vec3& v) {
        Body* r = getRoot();
        r->angularVelocity = v;
    }

    glm::vec3 Body::getVelocityAtPoint(const glm::vec3& worldPoint) const {
        const Body* r = getRoot();
        glm::vec3 rel = worldPoint - r->worldCFrame.translation;
        return r->linearVelocity + glm::cross(r->angularVelocity, rel);
    }

    void Body::accumulateForce(const glm::vec3& f) {
        if (staticBody) return;
        getRoot()->externalForce += f;
    }

    void Body::accumulateTorque(const glm::vec3& t) {
        if (staticBody) return;
        getRoot()->externalTorque += t;
    }

    void Body::accumulateForceAtPoint(const glm::vec3& f, const glm::vec3& worldPoint) {
        if (staticBody) return;
        Body* r = getRoot();
        r->externalForce += f;
        glm::vec3 rel = worldPoint - r->worldCFrame.translation;
        r->externalTorque += glm::cross(rel, f);
    }

    void Body::accumulateImpulse(const glm::vec3& i) {
        if (staticBody) return;
        getRoot()->externalImpulse += i;
    }

    void Body::accumulateImpulseAtPoint(const glm::vec3& i, const glm::vec3& worldPoint) {
        if (staticBody) return;
        Body* r = getRoot();
        r->externalImpulse += i;
        glm::vec3 rel = worldPoint - r->worldCFrame.translation;
        r->externalRotationalImpulse += glm::cross(rel, i);
    }

    void Body::accumulateRotationalImpulse(const glm::vec3& ri) {
        if (staticBody) return;
        getRoot()->externalRotationalImpulse += ri;
    }

    void Body::clearAccumulators() {
        Body* r = getRoot();
        r->externalForce = glm::vec3(0.0f);
        r->externalTorque = glm::vec3(0.0f);
        r->externalImpulse = glm::vec3(0.0f);
        r->externalRotationalImpulse = glm::vec3(0.0f);
    }

    void Body::markBranchDirty() {
        Body* r = getRoot();
        r->branchDirty = true;
    }

    glm::vec3 Body::getBranchIBodyV3() const {
        const Body* r = getRoot();
        if (r->branchDirty) const_cast<Body*>(r)->recomputeBranchProperties();
        return glm::vec3(
            r->branchInertiaLocal[0][0],
            r->branchInertiaLocal[1][1],
            r->branchInertiaLocal[2][2]
        );
    }

    void Body::recomputeBranchRecursive(const CoordinateFrame& rootCF, float& outMass, glm::vec3& outCofmWorld, glm::mat3& outInertiaWorld) const {
        CoordinateFrame myWorld = parent ? (parent->getWorldCFrame() * localCFrame) : worldCFrame;

        if (!staticBody && mass > 0.0f) {
            glm::vec3 myCofmWorld = myWorld.translation;
            outMass += mass;
            outCofmWorld += myCofmWorld * mass;

            // I_world = R * I_body * R^T  +  m * (|r|^2 * I - r*r^T)
            glm::mat3 R = myWorld.rotation;
            glm::mat3 Iworld = R * inertiaBody * glm::transpose(R);
            glm::vec3 r = myCofmWorld;  // from origin
            glm::mat3 shift = glm::mat3(0.0f);
            float rdotr = glm::dot(r, r);
            for (int i = 0; i < 3; i++) {
                shift[i][i] = rdotr;
                for (int j = 0; j < 3; j++) {
                    shift[i][j] -= r[i] * r[j];
                }
            }
            shift *= mass;
            outInertiaWorld += Iworld + shift;
        }

        for (Body* c : children) {
            c->recomputeBranchRecursive(rootCF, outMass, outCofmWorld, outInertiaWorld);
        }
    }

    void Body::recomputeBranchProperties() {
        if (parent) {
            getRoot()->recomputeBranchProperties();
            return;
        }
        if (!branchDirty) return;

        float totalMass = 0.0f;
        glm::vec3 cofmWorldAccum(0.0f);
        glm::mat3 inertiaWorldOrigin(0.0f);

        recomputeBranchRecursive(worldCFrame, totalMass, cofmWorldAccum, inertiaWorldOrigin);

        branchMass = totalMass;
        if (totalMass > 0.0f) {
            glm::vec3 cofmWorld = cofmWorldAccum / totalMass;
            branchCofmOffsetLocal = glm::transpose(worldCFrame.rotation) * (cofmWorld - worldCFrame.translation);

            // parallel-axis: shift inertia from world-origin to branch COM
            glm::vec3 r = cofmWorld;
            glm::mat3 shift = glm::mat3(0.0f);
            float rdotr = glm::dot(r, r);
            for (int i = 0; i < 3; i++) {
                shift[i][i] = rdotr;
                for (int j = 0; j < 3; j++) {
                    shift[i][j] -= r[i] * r[j];
                }
            }
            shift *= totalMass;
            glm::mat3 inertiaAboutCofm = inertiaWorldOrigin - shift;

            branchInertiaLocal = glm::transpose(worldCFrame.rotation) * inertiaAboutCofm * worldCFrame.rotation;
        } else {
            branchCofmOffsetLocal = glm::vec3(0.0f);
            branchInertiaLocal = glm::mat3(0.0f);
        }

        branchDirty = false;
    }

    glm::vec3 Body::getBranchCofmWorld() const {
        const Body* r = getRoot();
        if (r->branchDirty) const_cast<Body*>(r)->recomputeBranchProperties();
        return r->worldCFrame.translation + r->worldCFrame.rotation * r->branchCofmOffsetLocal;
    }

    glm::mat3 Body::getBranchInertiaWorldAtPoint(const glm::vec3& worldPoint) const {
        const Body* r = getRoot();
        if (r->branchDirty) const_cast<Body*>(r)->recomputeBranchProperties();

        glm::mat3 R = r->worldCFrame.rotation;
        glm::mat3 Iworld = R * r->branchInertiaLocal * glm::transpose(R);

        glm::vec3 cofmWorld = r->worldCFrame.translation + R * r->branchCofmOffsetLocal;
        glm::vec3 d = worldPoint - cofmWorld;
        glm::mat3 shift = glm::mat3(0.0f);
        float ddotd = glm::dot(d, d);
        for (int i = 0; i < 3; i++) {
            shift[i][i] = ddotd;
            for (int j = 0; j < 3; j++) {
                shift[i][j] -= d[i] * d[j];
            }
        }
        shift *= r->branchMass;
        return Iworld + shift;
    }

} // namespace OWSolver
