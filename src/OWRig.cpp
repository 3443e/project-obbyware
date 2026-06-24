#include "OWRig.hpp"
#include <glm/glm.hpp>

OWRig::OWRig() {
    // -------- Sizes -------- //
    rootPart.SetSize({2, 2, 1});
    torso.SetSize({2, 2, 1});
    head.SetSize({2, 1, 1});
    leftArm.SetSize({1, 2, 1});
    rightArm.SetSize({1, 2, 1});
    leftLeg.SetSize({1, 2, 1});
    rightLeg.SetSize({1, 2, 1});

    // -------- Collision -------- //
    rootPart.SetCanCollide(false);
    torso.SetCanCollide(true);
    head.SetCanCollide(true);
    leftArm.SetCanCollide(false);
    rightArm.SetCanCollide(false);
    leftLeg.SetCanCollide(false);
    rightLeg.SetCanCollide(false);

    // -------- Masses -------- //
    rootPart.SetMass(8.4f);
    torso.SetMass(4.0f);
    head.SetMass(2.0f);
    leftArm.SetMass(1.0f);
    rightArm.SetMass(1.0f);
    leftLeg.SetMass(1.0f);
    rightLeg.SetMass(1.0f);
    

    // -------- Friction -------- //
    rootPart.SetFriction(0.3f);
    torso.SetFriction(0.3f);
    head.SetFriction(0.3f);
    leftArm.SetFriction(0.3f);
    rightArm.SetFriction(0.3f);
    leftLeg.SetFriction(0.3f);
    rightLeg.SetFriction(0.3f);

    // -------- Colors -------- //
    rootPart.SetVisible(false); /* root part is invisible */ 
    torso.SetColor(RAYWHITE);
    head.SetColor(RAYWHITE);
    leftArm.SetColor(RAYWHITE);
    rightArm.SetColor(RAYWHITE);
    leftLeg.SetColor(RAYWHITE);
    rightLeg.SetColor(RAYWHITE);

    // -------- Weld children to root -------- //
    // local transforms = offset from root center to child center
    glm::mat3 I(1.0f);
    rootPart.getBody()->weldChild(torso.getBody(), OWSolver::CoordinateFrame(I, glm::vec3( 0.0f, 0.0f, 0.0f)));
    rootPart.getBody()->weldChild(head.getBody(), OWSolver::CoordinateFrame(I, glm::vec3( 0.0f, 1.5f, 0.0f)));
    rootPart.getBody()->weldChild(leftArm.getBody(), OWSolver::CoordinateFrame(I, glm::vec3( 1.5f, 0.0f, 0.0f)));
    rootPart.getBody()->weldChild(rightArm.getBody(), OWSolver::CoordinateFrame(I, glm::vec3(-1.5f, 0.0f, 0.0f)));
    rootPart.getBody()->weldChild(leftLeg.getBody(), OWSolver::CoordinateFrame(I, glm::vec3( 0.5f, -2.0f, 0.0f)));
    rootPart.getBody()->weldChild(rightLeg.getBody(), OWSolver::CoordinateFrame(I, glm::vec3(-0.5f, -2.0f, 0.0f)));

    SetPosition({0, 5, 0});
}

void OWRig::SetPosition(const Vector3& pos) {
    rootPart.SetPosition(pos);
}

Vector3 OWRig::GetPosition() const {
    return rootPart.GetPosition();
}

void OWRig::Render() {
    rootPart.Render();   // skips (invisible)
    torso.Render();
    head.Render();
    leftArm.Render();
    rightArm.Render();
    leftLeg.Render();
    rightLeg.Render();
}