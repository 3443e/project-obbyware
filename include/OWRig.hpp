#pragma once
#include "OWInstance/OWPart.hpp"
#include "OWAnimation.hpp"
#include <raylib.h>

class OWRig {
public:
    OWRig();
    void SetPosition(const Vector3& pos);
    Vector3 GetPosition() const;
    OWPart* getRoot() { return &rootPart; }
    void Render();

    void playAnimation(const OWAnimation* anim);
    void updateAnimation(float dt);
    const OWAnimation* getCurrentAnim() const { return currentAnim; }

private:
    //order matters btw
    OWPart rootPart;
    OWPart torso;
    OWPart head;
    OWPart leftArm;
    OWPart rightArm;
    OWPart leftLeg;
    OWPart rightLeg;

    OWSolver::CoordinateFrame baseTorso;
    OWSolver::CoordinateFrame baseHead;
    OWSolver::CoordinateFrame baseLeftArm;
    OWSolver::CoordinateFrame baseRightArm;
    OWSolver::CoordinateFrame baseLeftLeg;
    OWSolver::CoordinateFrame baseRightLeg;

    glm::mat3 c0RotArmsLegs;
    glm::mat3 c0RotHead;
    glm::mat3 c0RotLeftArmsLegs;

    glm::vec3 c1TransArm;
    glm::vec3 c1TransLeg;
    glm::vec3 c1TransHead;

    const OWAnimation* currentAnim = nullptr;
    float animTime = 0.0f;

    OWSolver::CoordinateFrame lerpCF(const OWSolver::CoordinateFrame& a, const OWSolver::CoordinateFrame& b, float t);
    OWSolver::CoordinateFrame getPose(const Keyframe& kf, const std::string& name);
    OWSolver::CoordinateFrame applyMotor6D(
    const OWSolver::CoordinateFrame& base,
    const glm::mat3& c0Rot,
    const glm::vec3& c1Trans,
    const OWSolver::CoordinateFrame& pose);
};