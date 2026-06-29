#pragma once
#include "OWInstance/OWPart.hpp"
#include "OWAnimation.hpp"
#include "libowsolver/ConstraintTypes.hpp"
#include <raylib.h>
#include <vector>

struct AnimTrack {
    enum State { Playing, Stopping };
    const OWAnimation* anim;
    float time;
    float weight;
    float targetWeight;
    float fadeSpeed;
    float speed;
    State state;
    bool paused;
};

struct LimbJoint {
    OWSolver::Body* parent = nullptr;
    OWSolver::Body* child = nullptr;
    OWSolver::CoordinateFrame c0;
    OWSolver::CoordinateFrame c1;
    OWSolver::ConstraintBallInSocket* pivot = nullptr;
    OWSolver::ConstraintBodyAngularVelocity* drive = nullptr;
    float kP = 0.0f;
    float kD = 0.0f;
    float maxTorque = 0.0f;
};

class OWRig {
public:
    OWRig();
    ~OWRig();

    void SetPosition(const Vector3& pos);
    Vector3 GetPosition() const;
    OWPart* getRoot() { return &rootPart; }
    std::vector<const OWSolver::Body*> getAllBodies() const;
    void Render();

    void playAnimation(const OWAnimation* anim, float fadeTime);
    void stopAnimation(const OWAnimation* anim, float fadeTime = 0.1f);
    void stopAllAnimations(float fadeTime = 0.1f);
    void setAnimationPaused(const OWAnimation* anim, bool paused);
    void setAnimationSpeed(const OWAnimation* anim, float speed);
    void SetTransparency(float t);
    void updateAnimation(float dt);
private:
    OWPart rootPart;
    OWPart torso;
    OWPart head;
    OWPart leftArm;
    OWPart rightArm;
    OWPart leftLeg;
    OWPart rightLeg;

    // C0 and C1 as full CoordinateFrames (rotation + translation)
    OWSolver::CoordinateFrame c0Torso, c1Torso;
    OWSolver::CoordinateFrame c0Head, c1Head;
    OWSolver::CoordinateFrame c0LeftArm, c1LeftArm;
    OWSolver::CoordinateFrame c0RightArm, c1RightArm;
    OWSolver::CoordinateFrame c0LeftLeg, c1LeftLeg;
    OWSolver::CoordinateFrame c0RightLeg, c1RightLeg;

    LimbJoint torsoJoint, headJoint, leftArmJoint, rightArmJoint, leftLegJoint, rightLegJoint;

    std::vector<AnimTrack> tracks;

    OWSolver::CoordinateFrame lerpCF(const OWSolver::CoordinateFrame& a, const OWSolver::CoordinateFrame& b, float t);
    OWSolver::CoordinateFrame getPose(const Keyframe& kf, const std::string& name);
    OWSolver::CoordinateFrame samplePose(const OWAnimation* anim, float time, const std::string& partName);
    OWSolver::CoordinateFrame applyMotor6D(
        const OWSolver::CoordinateFrame& c0,
        const OWSolver::CoordinateFrame& c1,
        const OWSolver::CoordinateFrame& pose);

    void addLimbJoint(LimbJoint& joint, OWPart* parent, OWPart* child, const OWSolver::CoordinateFrame& c0, const OWSolver::CoordinateFrame& c1, float kP, float kD, float maxTorque);
    void updateLimbJoint(LimbJoint& j, const OWSolver::CoordinateFrame& animPose);
};