#define GLM_ENABLE_EXPERIMENTAL
#include "OWRig.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <algorithm>

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

    glm::mat3 I(1.0f);
    glm::mat3 rot180Y = glm::mat3(
        -1, 0,  0,
         0, 1,  0,
         0, 0, -1
    );
    glm::mat3 rotNeg90X = glm::mat3(
        1,  0, 0,
        0,  0, -1,
        0,  1, 0
    );
    baseTorso = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, 0, 0));
    baseHead = OWSolver::CoordinateFrame(rotNeg90X, glm::vec3(0, 1.5f, 0));
    baseLeftArm = OWSolver::CoordinateFrame(I, glm::vec3(1.5f, 0, 0));
    baseRightArm = OWSolver::CoordinateFrame(I, glm::vec3(-1.5f, 0, 0));
    baseLeftLeg = OWSolver::CoordinateFrame(I, glm::vec3(0.5f, -2.0f, 0));
    baseRightLeg = OWSolver::CoordinateFrame(I, glm::vec3(-0.5f, -2.0f, 0));

    c0RotArmsLegs = glm::mat3(
        0, 0, -1,
        0, 1,  0,
        1, 0,  0
    );
    c0RotHead = rot180Y;

    c0RotLeftArmsLegs = glm::mat3(
        0, 0, 1,
        0, 1, 0,
       -1, 0, 0
    );

    c1TransArm  = glm::vec3(0, -0.5f, 0);
    c1TransLeg  = glm::vec3(0, -1.0f, 0);
    c1TransHead = glm::vec3(0, -0.5f, 0);

    rootPart.getBody()->weldChild(torso.getBody(), baseTorso);
    torso.getBody()->weldChild(head.getBody(), baseHead);
    torso.getBody()->weldChild(leftArm.getBody(), baseLeftArm);
    torso.getBody()->weldChild(rightArm.getBody(), baseRightArm);
    torso.getBody()->weldChild(leftLeg.getBody(), baseLeftLeg);
    torso.getBody()->weldChild(rightLeg.getBody(), baseRightLeg);

    SetPosition({0, 5, 0});
}

void OWRig::SetPosition(const Vector3& pos) { rootPart.SetPosition(pos); }
Vector3 OWRig::GetPosition() const { return rootPart.GetPosition(); }

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
    glm::mat3 rot = glm::mat3_cast(q);

    return OWSolver::CoordinateFrame(rot, pos);
}

OWSolver::CoordinateFrame OWRig::getPose(const Keyframe& kf, const std::string& name) {
    for (const auto& p : kf.poses) {
        if (p.name == name) return p.cframe;
    }
    return OWSolver::CoordinateFrame(glm::mat3(1.0f), glm::vec3(0));
}

OWSolver::CoordinateFrame OWRig::samplePose(const OWAnimation* anim, float time, const std::string& partName) {
    if (!anim || anim->keyframes.empty()) {
        return OWSolver::CoordinateFrame(glm::mat3(1.0f), glm::vec3(0));
    }

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

OWSolver::CoordinateFrame OWRig::applyMotor6D(const OWSolver::CoordinateFrame& base, const glm::mat3& c0Rot, const glm::vec3& c1Trans, const OWSolver::CoordinateFrame& pose) {
    glm::mat3 c0RotInv = glm::transpose(c0Rot);
    glm::mat3 finalRot = base.rotation * c0Rot * pose.rotation * c0RotInv;

    glm::vec3 poseTrans = pose.translation + (pose.rotation - glm::mat3(1.0f)) * c1Trans;
    glm::vec3 finalTrans = base.translation + base.rotation * c0Rot * poseTrans;

    return OWSolver::CoordinateFrame(finalRot, finalTrans);
}

void OWRig::updateAnimation(float dt) {
    for (auto& t : tracks) {
        if (t.state == AnimTrack::Playing && !t.paused) {
            t.time += dt * t.speed;
        }

        if (t.weight < t.targetWeight) {
            t.weight += t.fadeSpeed * dt;
            if (t.weight > t.targetWeight) t.weight = t.targetWeight;
        } else if (t.weight > t.targetWeight) {
            t.weight -= t.fadeSpeed * dt;
            if (t.weight < t.targetWeight) t.weight = t.targetWeight;
        }
    }

    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
        [](const AnimTrack& t) {
            return t.state == AnimTrack::Stopping && t.weight <= 0.0001f;
        }),
        tracks.end());

    if (tracks.empty()) return;

    auto blendPose = [&](const std::string& partName) -> OWSolver::CoordinateFrame {
        OWSolver::CoordinateFrame result(glm::mat3(1.0f), glm::vec3(0));
        float totalWeight = 0.0f;
        bool first = true;

        for (const auto& t : tracks) {
            if (t.weight <= 0.0001f) continue;

            OWSolver::CoordinateFrame pose = samplePose(t.anim, t.time, partName);

            if (first) {
                result = pose;
                totalWeight = t.weight;
                first = false;
            } else {
                float alpha = t.weight / (totalWeight + t.weight);
                result = lerpCF(result, pose, alpha);
                totalWeight += t.weight;
            }
        }
        return result;
    };

    // huge hack
    OWSolver::CoordinateFrame headPose = blendPose("Head");
    glm::mat3 forwardBase = glm::mat3(
        1,  0, 0,
        0,  0, -1,
        0,  1, 0
    );
    
    glm::quat poseQ = glm::quat_cast(headPose.rotation);
    glm::quat baseQ = glm::quat_cast(forwardBase);
    if (glm::dot(poseQ, baseQ) < 0.0f) poseQ = -poseQ;
    glm::quat finalQ = baseQ * poseQ;
    
    OWSolver::CoordinateFrame headFinal(
        glm::mat3_cast(finalQ),
        glm::vec3(0, 1.5f, 0) + headPose.translation
    );
    head.getBody()->setLocalCFrame(headFinal);

    OWSolver::CoordinateFrame leftArmPose = blendPose("Left Arm");
    leftArm.getBody()->setLocalCFrame(applyMotor6D(baseLeftArm, c0RotArmsLegs, c1TransArm, leftArmPose));

    OWSolver::CoordinateFrame rightArmPose = blendPose("Right Arm");
    rightArm.getBody()->setLocalCFrame(applyMotor6D(baseRightArm, c0RotLeftArmsLegs, c1TransArm, rightArmPose));

    OWSolver::CoordinateFrame leftLegPose = blendPose("Left Leg");
    leftLeg.getBody()->setLocalCFrame(applyMotor6D(baseLeftLeg, c0RotArmsLegs, c1TransLeg, leftLegPose));

    OWSolver::CoordinateFrame rightLegPose = blendPose("Right Leg");
    rightLeg.getBody()->setLocalCFrame(applyMotor6D(baseRightLeg, c0RotLeftArmsLegs, c1TransLeg, rightLegPose));
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