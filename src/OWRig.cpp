#define GLM_ENABLE_EXPERIMENTAL
#include "OWRig.hpp"
#include "OWWorld.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

static glm::vec3 rotationVectorBetween(const glm::mat3& from, const glm::mat3& to) {
    glm::quat qFrom = glm::quat_cast(from);
    glm::quat qTo = glm::quat_cast(to);
    glm::quat qError = qTo * glm::inverse(qFrom);
    if (qError.w < 0.0f) {
        qError = -qError;
    }
    float w = std::clamp(qError.w, -1.0f, 1.0f);
    float angle = 2.0f * std::acos(w);
    float s = std::sqrt(std::max(0.0f, 1.0f - w * w));
    if (s < 1e-6f) {
        return glm::vec3(0.0f);
    }
    return glm::vec3(qError.x, qError.y, qError.z) / s * angle;
}

OWRig::OWRig() {
    rootPart.SetSize({2, 2, 1});
    torso.SetSize({2, 2, 1});
    head.SetSize({2, 1, 1});
    leftArm.SetSize({1, 2, 1});
    rightArm.SetSize({1, 2, 1});
    leftLeg.SetSize({1, 2, 1});
    rightLeg.SetSize({1, 2, 1});

    rootPart.SetCanCollide(false);
    torso.SetCanCollide(true);
    head.SetCanCollide(true);
    leftArm.SetCanCollide(false);
    rightArm.SetCanCollide(false);
    leftLeg.SetCanCollide(false);
    rightLeg.SetCanCollide(false);

    rootPart.SetMass(8.4f);
    torso.SetMass(4.0f);
    head.SetMass(2.0f);
    leftArm.SetMass(1.0f);
    rightArm.SetMass(1.0f);
    leftLeg.SetMass(1.0f);
    rightLeg.SetMass(1.0f);

    rootPart.SetVisible(false);
    torso.SetColor(PURPLE);
    head.SetColor(RAYWHITE);
    leftArm.SetColor(RAYWHITE);
    rightArm.SetColor(RAYWHITE);
    leftLeg.SetColor(RAYWHITE);
    rightLeg.SetColor(RAYWHITE);

    rootPart.getBody()->setName("HumanoidRootPart");
    torso.getBody()->setName("Torso");
    head.getBody()->setName("Head");
    leftArm.getBody()->setName("Left Arm");
    rightArm.getBody()->setName("Right Arm");
    leftLeg.getBody()->setName("Left Leg");
    rightLeg.getBody()->setName("Right Leg");

    glm::mat3 rot180Y = glm::mat3(-1,0,0, 0,1,0, 0,0,-1);
    glm::mat3 rot90Y = glm::mat3(0,0,-1, 0,1,0, 1,0,0);
    glm::mat3 rotNeg90Y = glm::mat3(0,0,1, 0,1,0, -1,0,0);

    c0Torso = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, 0, 0));
    c1Torso = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, 0, 0));
    c0Head = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0,  1.0f, 0));
    c1Head = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, -0.5f, 0));
    c0LeftArm = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3(-1.0f, 0.5f, 0));
    c1LeftArm = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3( 0.5f, 0.5f, 0));
    c0RightArm = OWSolver::CoordinateFrame(rot90Y, glm::vec3( 1.0f, 0.5f, 0));
    c1RightArm = OWSolver::CoordinateFrame(rot90Y, glm::vec3(-0.5f, 0.5f, 0));
    c0LeftLeg = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3(-1.0f, -1.0f, 0));
    c1LeftLeg = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3(-0.5f,  1.0f, 0));
    c0RightLeg = OWSolver::CoordinateFrame(rot90Y, glm::vec3( 1.0f, -1.0f, 0));
    c1RightLeg = OWSolver::CoordinateFrame(rot90Y, glm::vec3( 0.5f,  1.0f, 0));


    OWSolver::CoordinateFrame rootCF(rot180Y, glm::vec3(0, 5, 0));
    rootPart.getBody()->setWorldCFrame(rootCF);

    OWSolver::CoordinateFrame rest; 
    OWSolver::CoordinateFrame torsoCF = rootCF * applyMotor6D(c0Torso, c1Torso, rest);
    torso.getBody()->setWorldCFrame(torsoCF);

    head.getBody()->setWorldCFrame(torsoCF * applyMotor6D(c0Head, c1Head, rest));
    leftArm.getBody()->setWorldCFrame(torsoCF * applyMotor6D(c0LeftArm, c1LeftArm, rest));
    rightArm.getBody()->setWorldCFrame(torsoCF * applyMotor6D(c0RightArm, c1RightArm, rest));
    leftLeg.getBody()->setWorldCFrame(torsoCF * applyMotor6D(c0LeftLeg, c1LeftLeg, rest));
    rightLeg.getBody()->setWorldCFrame(torsoCF * applyMotor6D(c0RightLeg, c1RightLeg, rest));


    addLimbJoint(torsoJoint, &rootPart, &torso, c0Torso, c1Torso, 200.0f, 25.0f, 8000.0f);
    addLimbJoint(headJoint, &torso, &head, c0Head, c1Head, 120.0f, 18.0f, 1500.0f);
    addLimbJoint(leftArmJoint, &torso, &leftArm, c0LeftArm, c1LeftArm, 90.0f, 14.0f,  800.0f);
    addLimbJoint(rightArmJoint, &torso, &rightArm, c0RightArm, c1RightArm, 90.0f, 14.0f,  800.0f);
    addLimbJoint(leftLegJoint, &torso, &leftLeg, c0LeftLeg, c1LeftLeg, 150.0f, 20.0f, 2500.0f);
    addLimbJoint(rightLegJoint, &torso, &rightLeg, c0RightLeg, c1RightLeg, 150.0f, 20.0f, 2500.0f);
}

OWRig::~OWRig() {
    LimbJoint* joints[] = { &torsoJoint, &headJoint, &leftArmJoint, &rightArmJoint, &leftLegJoint, &rightLegJoint };
    for (auto* j : joints) {
        if (OWWorld::Active) {
            OWWorld::Active->removePersistentConstraint(j->pivot);
            OWWorld::Active->removePersistentConstraint(j->drive);
        }
        delete j->pivot;
        delete j->drive;
    }
}


void OWRig::SetPosition(const Vector3& pos) {
    rootPart.SetPosition(pos);
}

Vector3 OWRig::GetPosition() const {
    return rootPart.GetPosition();
}

void OWRig::addLimbJoint(LimbJoint& joint, OWPart* parent, OWPart* child, const OWSolver::CoordinateFrame& c0, const OWSolver::CoordinateFrame& c1, float kP, float kD, float maxTorque) {
    joint.parent = parent->getBody();
    joint.child = child->getBody();
    joint.c0 = c0;
    joint.c1 = c1;
    joint.kP = kP;
    joint.kD = kD;
    joint.maxTorque = maxTorque;

    joint.pivot = new OWSolver::ConstraintBallInSocket(joint.parent, joint.child);
    joint.pivot->setPivotA(c0.translation);
    joint.pivot->setPivotB(c1.translation);

    joint.drive = new OWSolver::ConstraintBodyAngularVelocity(joint.parent, joint.child);
    joint.drive->setUseIntegratedVelocities(false);
    joint.drive->setMaxTorque(glm::vec3(maxTorque));
    joint.drive->setMinTorque(glm::vec3(-maxTorque));

    if (OWWorld::Active) {
        OWWorld::Active->addPersistentConstraint(joint.pivot);
        OWWorld::Active->addPersistentConstraint(joint.drive);
        OWWorld::Active->ignoreCollisionPair(joint.parent, joint.child);
    }
}

void OWRig::updateLimbJoint(LimbJoint& j, const OWSolver::CoordinateFrame& animPose) {
    OWSolver::CoordinateFrame parentWorld = j.parent->getWorldCFrame();
    OWSolver::CoordinateFrame childWorld = j.child->getWorldCFrame();

    OWSolver::CoordinateFrame targetChildWorld = parentWorld * applyMotor6D(j.c0, j.c1, animPose);

    glm::vec3 errorWorld = rotationVectorBetween(childWorld.rotation, targetChildWorld.rotation);
    glm::vec3 errorInParentFrame = glm::transpose(parentWorld.rotation) * errorWorld;

    glm::vec3 relAngVelWorld = j.child->getAngularVelocity() - j.parent->getAngularVelocity();
    glm::vec3 relAngVelInParentFrame = glm::transpose(parentWorld.rotation) * relAngVelWorld;

    glm::vec3 desiredAccel = j.kP * errorInParentFrame - j.kD * relAngVelInParentFrame;
    glm::vec3 targetRelAngVel = relAngVelInParentFrame + desiredAccel * (1.0f / 240.0f);

    j.drive->setTarget(-targetRelAngVel);
}

std::vector<const OWSolver::Body*> OWRig::getAllBodies() const {
    return { rootPart.getBody(), torso.getBody(), head.getBody(), leftArm.getBody(), rightArm.getBody(), leftLeg.getBody(), rightLeg.getBody() };
}

void OWRig::Render() {
    rootPart.Render();
    torso.Render();
    head.Render();
    leftArm.Render();
    rightArm.Render();
    leftLeg.Render();
    rightLeg.Render();
}

void OWRig::playAnimation(const OWAnimation* anim, float fadeTime) {
    if (!anim) return;
    if (fadeTime <= 0.0f) fadeTime = 0.1f;
    for (auto& t : tracks) {
        if (t.anim == anim) {
            if (t.state == AnimTrack::Stopping) {
                t.state = AnimTrack::Playing;
            }
            t.targetWeight = 1.0f;
            t.fadeSpeed = 1.0f / fadeTime;
            return;
        }
    }
    AnimTrack track;
    track.anim = anim;
    track.time = 0.0f;
    track.weight = 0.0f;
    track.targetWeight = 1.0f;
    track.fadeSpeed = 1.0f / fadeTime;
    track.speed = 1.0f;
    track.state = AnimTrack::Playing;
    track.paused = false;
    tracks.push_back(track);
}

void OWRig::stopAnimation(const OWAnimation* anim, float fadeTime) {
    if (!anim) return;
    if (fadeTime < 0.0f) fadeTime = 0.1f;
    for (auto& t : tracks) {
        if (t.anim == anim && t.state != AnimTrack::Stopping) {
            t.state = AnimTrack::Stopping;
            t.targetWeight = 0.0f;
            t.fadeSpeed = 1.0f / fadeTime;
        }
    }
}

void OWRig::stopAllAnimations(float fadeTime) {
    if (fadeTime < 0.0f) fadeTime = 0.1f;
    for (auto& t : tracks) {
        if (t.state != AnimTrack::Stopping) {
            t.state = AnimTrack::Stopping;
            t.targetWeight = 0.0f;
            t.fadeSpeed = 1.0f / fadeTime;
        }
    }
}

void OWRig::setAnimationPaused(const OWAnimation* anim, bool paused) {
    for (auto& t : tracks) {
        if (t.anim == anim) { 
            t.paused = paused;
        }
    }
}

void OWRig::setAnimationSpeed(const OWAnimation* anim, float speed) {
    for (auto& t : tracks) {
        if (t.anim == anim) {
            t.speed = speed;
        }
    }
}

OWSolver::CoordinateFrame OWRig::lerpCF(const OWSolver::CoordinateFrame& a, const OWSolver::CoordinateFrame& b, float t) {
    glm::vec3 pos = a.translation + (b.translation - a.translation) * t;
    glm::quat qA = glm::quat_cast(a.rotation);
    glm::quat qB = glm::quat_cast(b.rotation);
    if (glm::dot(qA, qB) < 0.0f) {
        qB = -qB;
    }
    glm::quat q = glm::slerp(qA, qB, t);
    return OWSolver::CoordinateFrame(glm::mat3_cast(q), pos);
}

OWSolver::CoordinateFrame OWRig::getPose(const Keyframe& kf, const std::string& name) {
    for (const auto& p : kf.poses) {
        if (p.name == name) {
            return p.cframe;
        }
    }
    return OWSolver::CoordinateFrame(glm::mat3(1.0f), glm::vec3(0));
}

OWSolver::CoordinateFrame OWRig::samplePose(const OWAnimation* anim, float time, const std::string& partName) {
    if (!anim || anim->keyframes.empty())
        return OWSolver::CoordinateFrame(glm::mat3(1.0f), glm::vec3(0));

    if (anim->loop && anim->duration > 0) {
        time = fmod(time, anim->duration);
        if (time < 0) time += anim->duration;
    } else if (time > anim->duration) {
        time = anim->duration;
    }

    const Keyframe* prev = &anim->keyframes[0];
    const Keyframe* next = &anim->keyframes[0];
    for (size_t i = 0; i < anim->keyframes.size() - 1; i++) {
        if (anim->keyframes[i + 1].time >= time) {
            prev = &anim->keyframes[i];
            next = &anim->keyframes[i + 1];
            break;
        }
    }

    float alpha = 0;
    if (next->time > prev->time) {
        alpha = std::clamp((time - prev->time) / (next->time - prev->time), 0.0f, 1.0f);
    }

    return lerpCF(getPose(*prev, partName), getPose(*next, partName), alpha);
}

OWSolver::CoordinateFrame OWRig::applyMotor6D(const OWSolver::CoordinateFrame& c0, const OWSolver::CoordinateFrame& c1, const OWSolver::CoordinateFrame& pose) {
    return c0 * pose * c1.inverse();
}

void OWRig::updateAnimation(float dt) {
    for (auto& t : tracks) {
        if (t.state == AnimTrack::Playing && !t.paused) {
            t.time += dt * t.speed;
        }
        if (t.weight < t.targetWeight) {t.weight += t.fadeSpeed * dt; if (t.weight > t.targetWeight) t.weight = t.targetWeight; }
        else if (t.weight > t.targetWeight) { t.weight -= t.fadeSpeed * dt; if (t.weight < t.targetWeight) t.weight = t.targetWeight; }
    }
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(), [](const AnimTrack& t) { return t.state == AnimTrack::Stopping && t.weight <= 0.0001f; }), tracks.end());

    auto blendPose = [&](const std::string& partName) -> OWSolver::CoordinateFrame {
        OWSolver::CoordinateFrame result;
        float totalWeight = 0.0f;
        bool first = true;
        for (const auto& t : tracks) {
            if (t.weight <= 0.0001f) continue;
            OWSolver::CoordinateFrame pose = samplePose(t.anim, t.time, partName);
            if (first) { result = pose; totalWeight = t.weight; first = false; }
            else {
                float alpha = t.weight / (totalWeight + t.weight);
                result = lerpCF(result, pose, alpha);
                totalWeight += t.weight;
            }
        }
        return result;
    };

    updateLimbJoint(torsoJoint, blendPose("Torso"));
    updateLimbJoint(headJoint, blendPose("Head"));
    updateLimbJoint(leftArmJoint, blendPose("Left Arm"));
    updateLimbJoint(rightArmJoint, blendPose("Right Arm"));
    updateLimbJoint(leftLegJoint, blendPose("Left Leg"));
    updateLimbJoint(rightLegJoint, blendPose("Right Leg"));
}

void OWRig::SetTransparency(float t) {
    rootPart.SetTransparency(t);
    torso.SetTransparency(t);
    head.SetTransparency(t);
    leftArm.SetTransparency(t);
    rightArm.SetTransparency(t);
    leftLeg.SetTransparency(t);
    rightLeg.SetTransparency(t);
}