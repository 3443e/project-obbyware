#include "OWInstance/OWPart.hpp"
#include "OWWorld.hpp"
#include "Lighting/OWShaders.hpp"
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

    rlSetTexture(studded ? OWShaders::studTexture.id : OWShaders::grayTexture.id);

    unsigned char alpha = (unsigned char)(255.0f * (1.0f - transparency));
    Color renderColor = color;
    renderColor.a = alpha;

    switch (body->getShape()) {
       case OWSolver::Body::Shape_Box: {
            rlSetTexture(studded ? OWShaders::studTexture.id : OWShaders::grayTexture.id);
            float hx = size.x * 0.5f;
            float hy = size.y * 0.5f;
            float hz = size.z * 0.5f;
            
            rlBegin(RL_TRIANGLES);
                rlColor4ub(renderColor.r, renderColor.g, renderColor.b, renderColor.a);
                
                rlNormal3f(1, 0, 0);
                rlTexCoord2f(0, 0); rlVertex3f(hx, -hy,  hz);
                rlTexCoord2f(size.z, 0); rlVertex3f(hx, -hy, -hz);
                rlTexCoord2f(size.z, size.y); rlVertex3f(hx,  hy, -hz);
                rlTexCoord2f(0, 0); rlVertex3f(hx, -hy,  hz);
                rlTexCoord2f(size.z, size.y); rlVertex3f(hx,  hy, -hz);
                rlTexCoord2f(0, size.y); rlVertex3f(hx,  hy,  hz);

                rlNormal3f(-1, 0, 0);
                rlTexCoord2f(0, 0); rlVertex3f(-hx, -hy, -hz);
                rlTexCoord2f(size.z, 0); rlVertex3f(-hx, -hy,  hz);
                rlTexCoord2f(size.z, size.y); rlVertex3f(-hx,  hy,  hz);
                rlTexCoord2f(0, 0); rlVertex3f(-hx, -hy, -hz);
                rlTexCoord2f(size.z, size.y); rlVertex3f(-hx,  hy,  hz);
                rlTexCoord2f(0, size.y); rlVertex3f(-hx,  hy, -hz);

                rlNormal3f(0, 1, 0);
                rlTexCoord2f(0, 0);  rlVertex3f(-hx, hy,  hz);
                rlTexCoord2f(size.x, 0); rlVertex3f( hx, hy,  hz);
                rlTexCoord2f(size.x, size.z); rlVertex3f( hx, hy, -hz);
                rlTexCoord2f(0, 0); rlVertex3f(-hx, hy,  hz);
                rlTexCoord2f(size.x, size.z); rlVertex3f( hx, hy, -hz);
                rlTexCoord2f(0, size.z); rlVertex3f(-hx, hy, -hz);
                
                rlNormal3f(0, -1, 0);
                rlTexCoord2f(0, 0); rlVertex3f(-hx, -hy, -hz);
                rlTexCoord2f(size.x, 0); rlVertex3f( hx, -hy, -hz);
                rlTexCoord2f(size.x, size.z); rlVertex3f( hx, -hy,  hz);
                rlTexCoord2f(0, 0); rlVertex3f(-hx, -hy, -hz);
                rlTexCoord2f(size.x, size.z); rlVertex3f( hx, -hy,  hz);
                rlTexCoord2f(0, size.z); rlVertex3f(-hx, -hy,  hz);

                rlNormal3f(0, 0, 1);
                rlTexCoord2f(0, 0); rlVertex3f(-hx, -hy, hz);
                rlTexCoord2f(size.x, 0); rlVertex3f( hx, -hy, hz);
                rlTexCoord2f(size.x, size.y); rlVertex3f( hx,  hy, hz);
                rlTexCoord2f(0, 0); rlVertex3f(-hx, -hy, hz);
                rlTexCoord2f(size.x, size.y); rlVertex3f( hx,  hy, hz);
                rlTexCoord2f(0, size.y); rlVertex3f(-hx,  hy, hz);
  
                rlNormal3f(0, 0, -1);
                rlTexCoord2f(0, 0); rlVertex3f( hx, -hy, -hz);
                rlTexCoord2f(size.x, 0); rlVertex3f(-hx, -hy, -hz);
                rlTexCoord2f(size.x, size.y); rlVertex3f(-hx,  hy, -hz);
                rlTexCoord2f(0, 0); rlVertex3f( hx, -hy, -hz);
                rlTexCoord2f(size.x, size.y); rlVertex3f(-hx,  hy, -hz);
                rlTexCoord2f(0, size.y); rlVertex3f( hx,  hy, -hz);
            rlEnd();
            break;
        }
            
        case OWSolver::Body::Shape_Sphere: {
            float radius = size.x * 0.5f;
            DrawSphere({0, 0, 0}, radius, renderColor);
            break;
        }
        
        case OWSolver::Body::Shape_Cylinder: {
            float halfLen = size.x * 0.5f;
            float radius = size.y * 0.5f;
            int segments = 24;
            
            rlBegin(RL_TRIANGLES);
                rlColor4ub(renderColor.r, renderColor.g, renderColor.b, renderColor.a);
                
                for (int i = 0; i < segments; i++) {
                    float a1 = (float)i / segments * 2.0f * 3.14159265f;
                    float a2 = (float)(i + 1) / segments * 2.0f * 3.14159265f;
                    float y1 = cosf(a1) * radius, z1 = sinf(a1) * radius;
                    float y2 = cosf(a2) * radius, z2 = sinf(a2) * radius;
                    float ny1 = cosf(a1), nz1 = sinf(a1);
                    float ny2 = cosf(a2), nz2 = sinf(a2);
                    
                    rlNormal3f(ny1, 0, nz1);
                    rlVertex3f(-halfLen, y1, z1);
                    rlNormal3f(ny2, 0, nz2);
                    rlVertex3f(-halfLen, y2, z2);
                    rlNormal3f(ny2, 0, nz2);
                    rlVertex3f(halfLen, y2, z2);
                    
                    rlNormal3f(ny1, 0, nz1);
                    rlVertex3f(-halfLen, y1, z1);
                    rlNormal3f(ny2, 0, nz2);
                    rlVertex3f(halfLen, y2, z2);
                    rlNormal3f(ny1, 0, nz1);
                    rlVertex3f(halfLen, y1, z1);
                }
                
                rlNormal3f(-1, 0, 0);
                for (int i = 0; i < segments; i++) {
                    float a1 = (float)i / segments * 2.0f * 3.14159265f;
                    float a2 = (float)(i + 1) / segments * 2.0f * 3.14159265f;
                    rlVertex3f(-halfLen, 0, 0);
                    rlVertex3f(-halfLen, cosf(a2) * radius, sinf(a2) * radius);
                    rlVertex3f(-halfLen, cosf(a1) * radius, sinf(a1) * radius);
                }
                
                rlNormal3f(1, 0, 0);
                for (int i = 0; i < segments; i++) {
                    float a1 = (float)i / segments * 2.0f * 3.14159265f;
                    float a2 = (float)(i + 1) / segments * 2.0f * 3.14159265f;
                    rlVertex3f(halfLen, 0, 0);
                    rlVertex3f(halfLen, cosf(a1) * radius, sinf(a1) * radius);
                    rlVertex3f(halfLen, cosf(a2) * radius, sinf(a2) * radius);
                }
            rlEnd();
            break;
        }
        
        case OWSolver::Body::Shape_Wedge: {
            glm::vec3 h = {size.x * 0.5f, size.y * 0.5f, size.z * 0.5f};
            
            float slopeLen = sqrtf(h.y * h.y + h.z * h.z);
            float slopeNY = h.z / slopeLen;
            float slopeNZ = h.y / slopeLen;
            
            rlBegin(RL_TRIANGLES);
                rlColor4ub(renderColor.r, renderColor.g, renderColor.b, renderColor.a);

                // Bottom (Normal -Y)
                rlNormal3f(0, -1, 0);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y, -h.z);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
                // Front Wall (Normal -Z)
                rlNormal3f(0, 0, -1);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f(-h.x,  h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f( h.x, -h.y, -h.z);
                // Left Wall (Normal -X)
                rlNormal3f(-1, 0, 0);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f(-h.x,  h.y, -h.z); rlVertex3f(-h.x, -h.y, -h.z);
                // Right Wall (Normal +X)
                rlNormal3f(1, 0, 0);
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
                // Slope (Normal +Y/+Z)
                rlNormal3f(0, slopeNY, slopeNZ);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f(-h.x,  h.y, -h.z);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
            rlEnd();
            break;
        }

        case OWSolver::Body::Shape_CornerWedge: {
            glm::vec3 h = {size.x * 0.5f, size.y * 0.5f, size.z * 0.5f};

            float slope1Len = sqrtf(h.y * h.y + h.z * h.z);
            float slope1NY = h.z / slope1Len;
            float slope1NZ = -h.y / slope1Len;
            
            float slope2Len = sqrtf(h.y * h.y + h.x * h.x);
            float slope2NY = h.x / slope2Len;
            float slope2NX = h.y / slope2Len;
            
            rlBegin(RL_TRIANGLES);
                rlColor4ub(renderColor.r, renderColor.g, renderColor.b, renderColor.a);

                // Bottom (Normal -Y)
                rlNormal3f(0, -1, 0);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z); rlVertex3f(-h.x, -h.y,  h.z);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
                // Back (Normal -Z)
                rlNormal3f(0, 0, -1);
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z);
                // Left (Normal -X)
                rlNormal3f(-1, 0, 0);
                rlVertex3f(-h.x, -h.y, -h.z); rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
                // Front slope (Normal +Y, +Z direction)
                rlNormal3f(0, slope1NY, slope1NZ);
                rlVertex3f(-h.x, -h.y,  h.z); rlVertex3f( h.x, -h.y,  h.z); rlVertex3f( h.x,  h.y, -h.z);
                // Right slope (Normal +Y, +X direction)
                rlNormal3f(slope2NX, slope2NY, 0);
                rlVertex3f( h.x, -h.y, -h.z); rlVertex3f( h.x,  h.y, -h.z); rlVertex3f( h.x, -h.y,  h.z);
            rlEnd();
            break;
        }
        case OWSolver::Body::Shape_None:
            break;
    }
    
    //rlDrawRenderBatchActive();
    rlPopMatrix();
}