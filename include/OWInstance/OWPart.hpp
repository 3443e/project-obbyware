#pragma once
#include "OWInstance/OWInstance.hpp"
#include "libowsolver/Body.hpp"
#include <raylib.h>

class OWPart : public OWInstance {
public:
    OWPart();
    ~OWPart();

    Vector3 GetPosition() const;
    Vector3 GetSize() const {
        return size;
    }

    void SetPosition(const Vector3& pos);
    void SetOrientationEuler(const Vector3& eulerDegrees);

    void SetSize(const Vector3& s);
    void SetAnchored(bool anchored);
    void SetMass(float mass);
    void SetFriction(float f);
    void SetRestitution(float r);
    void SetCanCollide(bool c);

    // rendering
    void SetVisible(bool v) { visible = v; }
    void SetColor(Color c) { color = c; }
    void SetStudded(bool v) { studded = v; }
    void SetTransparency(float t) { transparency = t; }
    
    void SetTruss(bool t);
    void SetShapeBall();
    void SetShapeCylinder();
    void SetShapeWedge();
    void SetShapeCornerWedge();

    /* direct body access for freaky stuff */
    OWSolver::Body* getBody() { return body; }
    const OWSolver::Body* getBody() const { return body; }

    void Render() override;

private:
    OWSolver::Body* body = nullptr;
    Vector3 size = {1, 1, 1};
    bool visible = true;
    float transparency = 0.0f;

    bool studded = false;
    Color color = WHITE;
};