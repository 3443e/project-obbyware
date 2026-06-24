#include "OWRig.hpp"
#include <glm/glm.hpp>
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
    torso.SetColor(Color{30, 100, 200, 255});
    head.SetColor(Color{240, 200, 100, 255});
    leftArm.SetColor(Color{240, 200, 100, 255});
    rightArm.SetColor(Color{240, 200, 100, 255});
    leftLeg.SetColor(Color{40, 40, 40, 255});
    rightLeg.SetColor(Color{40, 40, 40, 255});

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
    baseTorso = OWSolver::CoordinateFrame(rot180Y, glm::vec3(0, 0, 0));
    baseHead = OWSolver::CoordinateFrame(I, glm::vec3(0, 1.5f, 0));
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

void OWRig::SetPosition(const Vector3& pos) {
    rootPart.SetPosition(pos);
}

Vector3 OWRig::GetPosition() const {
    return rootPart.GetPosition();
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

void OWRig::playAnimation(const OWAnimation* anim) {
    currentAnim = anim;
    animTime = 0.0f;
}

OWSolver::CoordinateFrame OWRig::lerpCF(const OWSolver::CoordinateFrame& a, const OWSolver::CoordinateFrame& b, float t) {
    glm::vec3 pos = a.translation + (b.translation - a.translation) * t;
    glm::mat3 rot = a.rotation + (b.rotation - a.rotation) * t;
    glm::vec3 c0 = glm::normalize(rot[0]);
    glm::vec3 c2 = glm::normalize(glm::cross(c0, rot[1]));
    glm::vec3 c1 = glm::cross(c2, c0);
    return OWSolver::CoordinateFrame(glm::mat3(c0, c1, c2), pos);
}

OWSolver::CoordinateFrame OWRig::getPose(const Keyframe& kf, const std::string& name) {
    for (const auto& p : kf.poses) {
        if (p.name == name) return p.cframe;
    }
    return OWSolver::CoordinateFrame(glm::mat3(1.0f), glm::vec3(0));
}

OWSolver::CoordinateFrame OWRig::applyMotor6D(const OWSolver::CoordinateFrame& base, const glm::mat3& c0Rot, const glm::vec3& c1Trans, const OWSolver::CoordinateFrame& pose) {
    glm::mat3 c0RotInv = glm::transpose(c0Rot);
    glm::mat3 finalRot = c0Rot * pose.rotation * c0RotInv;

    glm::vec3 poseTrans = pose.translation + (pose.rotation - glm::mat3(1.0f)) * c1Trans;
    glm::vec3 finalTrans = base.translation + c0Rot * poseTrans;

    return OWSolver::CoordinateFrame(finalRot, finalTrans);
}

void OWRig::updateAnimation(float dt) {
    if (!currentAnim || currentAnim->keyframes.empty()) return;

    animTime += dt;
    if (currentAnim->loop && currentAnim->duration > 0) {
        animTime = fmod(animTime, currentAnim->duration);
    } else if (animTime > currentAnim->duration) {
        animTime = currentAnim->duration;
    }

    const Keyframe* prev = &currentAnim->keyframes[0];
    const Keyframe* next = &currentAnim->keyframes[0];
    for (size_t i = 0; i < currentAnim->keyframes.size() - 1; i++) {
        if (currentAnim->keyframes[i + 1].time >= animTime) {
            prev = &currentAnim->keyframes[i];
            next = &currentAnim->keyframes[i + 1];
            break;
        }
    }

    float alpha = 0;
    if (next->time > prev->time) {
        alpha = std::clamp((animTime - prev->time) / (next->time - prev->time), 0.0f, 1.0f);
    }

    OWSolver::CoordinateFrame headPose = lerpCF(getPose(*prev, "Head"), getPose(*next, "Head"), alpha);
    head.getBody()->setLocalCFrame(applyMotor6D(baseHead, c0RotHead, c1TransHead, headPose));

    OWSolver::CoordinateFrame leftArmPose = lerpCF(getPose(*prev, "Left Arm"), getPose(*next, "Left Arm"), alpha);
    leftArm.getBody()->setLocalCFrame(applyMotor6D(baseLeftArm, c0RotArmsLegs, c1TransArm, leftArmPose));

    OWSolver::CoordinateFrame rightArmPose = lerpCF(getPose(*prev, "Right Arm"), getPose(*next, "Right Arm"), alpha);
    rightArm.getBody()->setLocalCFrame(applyMotor6D(baseRightArm, c0RotLeftArmsLegs, c1TransArm, rightArmPose));

    OWSolver::CoordinateFrame leftLegPose = lerpCF(getPose(*prev, "Left Leg"), getPose(*next, "Left Leg"), alpha);
    leftLeg.getBody()->setLocalCFrame(applyMotor6D(baseLeftLeg, c0RotArmsLegs, c1TransLeg, leftLegPose));

    OWSolver::CoordinateFrame rightLegPose = lerpCF(getPose(*prev, "Right Leg"), getPose(*next, "Right Leg"), alpha);
    rightLeg.getBody()->setLocalCFrame(applyMotor6D(baseRightLeg, c0RotLeftArmsLegs, c1TransLeg, rightLegPose));
}
