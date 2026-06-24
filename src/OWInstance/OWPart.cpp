#include "OWInstance/OWPart.hpp"
#include "OWWorld.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <raylib.h>
#include <rlgl.h>
#include <cmath>

static inline glm::vec3 toGlm(const Vector3& v) { return glm::vec3(v.x, v.y, v.z); }
static inline Vector3 toRaylib(const glm::vec3& v) { return Vector3{v.x, v.y, v.z}; }

static glm::mat3 eulerAnglesToMat3(Vector3 eulerDegrees) {
    float rx = glm::radians(eulerDegrees.x);
    float ry = glm::radians(eulerDegrees.y);
    float rz = glm::radians(eulerDegrees.z);
    float cx = std::cos(rx), sx = std::sin(rx);
    float cy = std::cos(ry), sy = std::sin(ry);
    float cz = std::cos(rz), sz = std::sin(rz);
    return glm::mat3(
        glm::vec3(cy*cz, sx*sy*cz + cx*sz, -cx*sy*cz + sx*sz),
        glm::vec3(-cy*sz, -sx*sy*sz + cx*cz, cx*sy*sz + sx*cz),
        glm::vec3(sy, -sx*cy, cx*cy)
    );
}

OWPart::OWPart() {
    if (OWWorld::Active) {
        body = OWWorld::Active->createDynamicBox(glm::vec3(0), glm::vec3(1), 1.0f);
    }
}

OWPart::~OWPart() {}

Vector3 OWPart::GetPosition() const {
    if (!body) return {0, 0, 0};
    return toRaylib(body->getWorldPosition());
}

void OWPart::SetPosition(const Vector3& pos) {
    if (!body) return;
    OWSolver::CoordinateFrame cf(body->getWorldOrientation(), toGlm(pos));
    body->setWorldCFrame(cf);
}

void OWPart::SetOrientationEuler(const Vector3& eulerDegrees) {
    if (!body) return;
    OWSolver::CoordinateFrame cf(eulerAnglesToMat3(eulerDegrees), body->getWorldPosition());
    body->setWorldCFrame(cf);
}

void OWPart::SetSize(const Vector3& s) {
    size = s;
    if (!body) return;
    float m = body->getMass();
    if (m > 0.0f && !body->isStatic()) {
        body->setBoxMassAndInertia(m, toGlm(s));
    } else {
        body->setSize(toGlm(s));
    }
    if (OWWorld::Active) {
        OWWorld::Active->updateBodyShape(body);
    }
}

void OWPart::SetAnchored(bool anchored) {
    if (!body) return;
    body->setStatic(anchored);
}

void OWPart::SetMass(float mass) {
    if (!body || body->isStatic()) return;
    body->setBoxMassAndInertia(mass, toGlm(size));
}

void OWPart::SetTruss(bool t) {
    if (body) body->setTruss(t);
}
void OWPart::SetShapeBall() {
    if (body) {
        body->setShape(OWSolver::Body::Shape_Sphere);
        float r = size.x * 0.5f;
        body->setSize(glm::vec3(r * 2, r * 2, r * 2));
        if (OWWorld::Active) OWWorld::Active->updateBodyShape(body);
    }
}

void OWPart::SetShapeCylinder() {
    if (body) {
        body->setShape(OWSolver::Body::Shape_Cylinder);
        if (OWWorld::Active) OWWorld::Active->updateBodyShape(body);
    }
}

void OWPart::SetShapeWedge() {
    if (body) {
        body->setShape(OWSolver::Body::Shape_Wedge);
        if (OWWorld::Active) OWWorld::Active->updateBodyShape(body);
    }
}

void OWPart::SetShapeCornerWedge() {
    if (body) {
        body->setShape(OWSolver::Body::Shape_CornerWedge);
        if (OWWorld::Active) OWWorld::Active->updateBodyShape(body);
    }
}

void OWPart::SetFriction(float f) {
    if (!body) return;
    body->setFriction(f);
}

void OWPart::SetRestitution(float r) {
    if (!body) return;
    body->setRestitution(r);
}

void OWPart::SetCanCollide(bool c) {
    if (!body) return;
    body->setCanCollide(c);
    // add orremove from bullet so collision detection reflects the new state.
    if (OWWorld::Active) {
        if (c) {
            OWWorld::Active->addBodyToCollision(body);
        } else {
            OWWorld::Active->removeBodyFromCollision(body);
        }
    }
}

void OWPart::Render() {
    if (!body || !visible) return;

    glm::vec3 pos = body->getWorldPosition();
    glm::mat3 rot = body->getWorldOrientation();

    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);

    float matrix[16] = {
        rot[0][0], rot[0][1], rot[0][2], 0.0f,
        rot[1][0], rot[1][1], rot[1][2], 0.0f,
        rot[2][0], rot[2][1], rot[2][2], 0.0f,
        0.0f,      0.0f,      0.0f,      1.0f
    };
    rlMultMatrixf(matrix);

    switch (body->getShape()) {
        case OWSolver::Body::Shape_Box:
            DrawCube({0, 0, 0}, size.x, size.y, size.z, color);
            DrawCubeWires({0, 0, 0}, size.x, size.y, size.z, BLACK);
            break;
            
        case OWSolver::Body::Shape_Sphere: {
            float radius = size.x * 0.5f;
            DrawSphere({0, 0, 0}, radius, color);
            DrawSphereWires({0, 0, 0}, radius, 16, 16, BLACK);
            break;
        }
        
        case OWSolver::Body::Shape_Cylinder: {
            float halfLen = size.x * 0.5f;
            float radius = size.y * 0.5f;
            Vector3 start = {-halfLen, 0, 0};
            Vector3 end = { halfLen, 0, 0};
            DrawCylinderEx(start, end, radius, radius, 16, color);
            DrawCylinderWiresEx(start, end, radius, radius, 16, BLACK);
            break;
        }
        
        /* chatgpt'd */
        case OWSolver::Body::Shape_Wedge: {
            glm::vec3 h = {size.x * 0.5f, size.y * 0.5f, size.z * 0.5f};
            rlBegin(RL_TRIANGLES);
                rlColor4ub(color.r, color.g, color.b, color.a);

                // 1. Bottom (Normal -Y)
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y, -h.z);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
                // 2. Front Wall (Normal -Z)
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f(-h.x,  h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f( h.x, -h.y, -h.z);
                // 3. Left Wall (Normal -X)
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f(-h.x,  h.y, -h.z); rlVertex3f(-h.x, -h.y, -h.z);
                // 4. Right Wall (Normal +X)
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
                // 5. Slope (Normal +Y, +Z)
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f(-h.x,  h.y, -h.z);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
            rlEnd();

            // Draw wires for the edges
            rlBegin(RL_LINES);
                rlColor4ub(BLACK.r, BLACK.g, BLACK.b, BLACK.a);
                
                // Bottom rectangle
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y,  h.z);
                rlVertex3f( h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y, -h.z);
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f(-h.x, -h.y, -h.z);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f(-h.x, -h.y,  h.z);
                
                // Front rectangle top edge
                rlVertex3f(-h.x,  h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                
                // Vertical edges
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f(-h.x,  h.y, -h.z);
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                
                // Slope edges
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f(-h.x,  h.y, -h.z);
                rlVertex3f( h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
            rlEnd();
            break;
        }

        case OWSolver::Body::Shape_CornerWedge: {
            glm::vec3 h = {size.x * 0.5f, size.y * 0.5f, size.z * 0.5f};
            rlBegin(RL_TRIANGLES);
                rlColor4ub(color.r, color.g, color.b, color.a);

                // Bottom
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z); rlVertex3f(-h.x, -h.y,  h.z);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
                // Back
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                // Left
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
                // Front
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
                // Right
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
            rlEnd();

            rlBegin(RL_LINES);
                rlColor4ub(BLACK.r, BLACK.g, BLACK.b, BLACK.a);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y, -h.z);
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
                rlVertex3f( h.x, -h.y,  h.z); rlVertex3f(-h.x, -h.y,  h.z);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f(-h.x, -h.y, -h.z);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                rlVertex3f( h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
            rlEnd();
            break;
        }
        case OWSolver::Body::Shape_None:
            break;
    }

    rlPopMatrix();
}