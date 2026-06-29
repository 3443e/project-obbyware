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

    glm::mat3 rot180Y = glm::mat3(
        -1, 0,  0,
         0, 1,  0,
         0, 0, -1
    );
    glm::mat3 rot90Y = glm::mat3(
         0, 0, -1,
         0, 1,  0,
         1, 0,  0
    );
    glm::mat3 rotNeg90Y = glm::mat3(
         0, 0,  1,
         0, 1,  0,
        -1, 0,  0
    );

    c0Torso = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, 0, 0));
    c1Torso = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, 0, 0));

    c0Head = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0,  1.0f, 0));
    c1Head = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, -0.5f, 0));

    c0LeftArm  = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3(-1.0f, 0.5f, 0));
    c1LeftArm  = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3( 0.5f, 0.5f, 0));

    c0RightArm = OWSolver::CoordinateFrame(rot90Y, glm::vec3( 1.0f, 0.5f, 0));
    c1RightArm = OWSolver::CoordinateFrame(rot90Y, glm::vec3(-0.5f, 0.5f, 0));

    c0LeftLeg  = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3(-1.0f, -1.0f, 0));
    c1LeftLeg  = OWSolver::CoordinateFrame(rotNeg90Y, glm::vec3(-0.5f,  1.0f, 0));

    c0RightLeg = OWSolver::CoordinateFrame(rot90Y, glm::vec3( 1.0f, -1.0f, 0));
    c1RightLeg = OWSolver::CoordinateFrame(rot90Y, glm::vec3( 0.5f,  1.0f, 0));

    rootPart.getBody()->weldChild(torso.getBody(), c0Torso * c1Torso.inverse());
    torso.getBody()->weldChild(head.getBody(), c0Head * c1Head.inverse());
    torso.getBody()->weldChild(leftArm.getBody(), c0LeftArm * c1LeftArm.inverse());
    torso.getBody()->weldChild(rightArm.getBody(), c0RightArm * c1RightArm.inverse());
    torso.getBody()->weldChild(leftLeg.getBody(), c0LeftLeg * c1LeftLeg.inverse());
    torso.getBody()->weldChild(rightLeg.getBody(), c0RightLeg * c1RightLeg.inverse());

    rootPart.getBody()->setWorldCFrame(OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, 5, 0)));
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
    return OWSolver::CoordinateFrame(glm::mat3_cast(q), pos);
}

OWSolver::CoordinateFrame OWRig::getPose(const Keyframe& kf, const std::string& name) {
    for (const auto& p : kf.poses) {
        if (p.name == name) return p.cframe;
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
        if (t.state == AnimTrack::Playing && !t.paused)
            t.time += dt * t.speed;

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
        }), tracks.end());

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

    torso.getBody()->setLocalCFrame(applyMotor6D(c0Torso, c1Torso, blendPose("Torso")));
    head.getBody()->setLocalCFrame(applyMotor6D(c0Head, c1Head, blendPose("Head")));
    leftArm.getBody()->setLocalCFrame(applyMotor6D(c0LeftArm, c1LeftArm, blendPose("Left Arm")));
    rightArm.getBody()->setLocalCFrame(applyMotor6D(c0RightArm, c1RightArm, blendPose("Right Arm")));
    leftLeg.getBody()->setLocalCFrame(applyMotor6D(c0LeftLeg, c1LeftLeg, blendPose("Left Leg")));
    rightLeg.getBody()->setLocalCFrame(applyMotor6D(c0RightLeg, c1RightLeg, blendPose("Right Leg")));
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