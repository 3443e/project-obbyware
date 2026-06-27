#include "OWCamera.hpp"
#include "OWInstance/OWContainer.hpp"
#include "OWWorld.hpp"
#include <functional>
#include <cmath>
#include <algorithm>

// glm::vec3 to raylib Vector3 helper conversion function
static inline Vector3 toRaylib(const glm::vec3& v) {
    return Vector3{v.x, v.y, v.z};
}

OWCamera::OWCamera() {
    cam.up = {0, 1, 0};
    cam.fovy = 70.0f;
    cam.projection = CAMERA_PERSPECTIVE;
}

void OWCamera::setShiftLock(bool enabled) {
    if (shiftLock == enabled) return;
    shiftLock = enabled;
    if (shiftLock) {
        DisableCursor();
    } else if (!rightMouseDown) {
        EnableCursor();
    }
}

void OWCamera::toggleShiftLock() {
    setShiftLock(!shiftLock);
}

glm::vec3 OWCamera::getFocusPoint() const {
    if (!target || !target->getBody()) return glm::vec3(0);
    return target->getBody()->getWorldPosition() + glm::vec3(0, 1.5f, 0);
}

glm::vec3 OWCamera::getLookDirection() const {
    float cp = std::cos(pitch);
    return glm::vec3(
        -std::sin(heading) * cp,
        std::sin(pitch),
        -std::cos(heading) * cp
    );
}

glm::vec3 OWCamera::getLookHorizontal() const {
    return glm::vec3(-std::sin(heading), 0, -std::cos(heading));
}

void OWCamera::updateInput(float dt) {
    (void)dt; // ignore

    if (IsKeyPressed(KEY_LEFT_SHIFT)) {
        toggleShiftLock();
    }

    bool rightDownNow = IsMouseButtonDown(MOUSE_RIGHT_BUTTON);
    if (rightDownNow && !rightMouseDown) {
        rightMouseDown = true;
        DisableCursor();
    } else if (!rightDownNow && rightMouseDown) {
        rightMouseDown = false;
        if (!shiftLock) {
            EnableCursor();
        }
    }

    bool rotateCamera = shiftLock || rightMouseDown;
    if (rotateCamera) {
        Vector2 mouseDelta = GetMouseDelta();
        heading -= mouseDelta.x * MOUSE_SENSITIVITY;
        pitch -= mouseDelta.y * MOUSE_SENSITIVITY;
        pitch = std::clamp(pitch, PITCH_MIN, PITCH_MAX);
    }

    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) {
        distance -= wheel * ZOOM_SPEED;
        distance = std::clamp(distance, DISTANCE_MIN, DISTANCE_MAX);
    }
}

void OWCamera::updatePosition() {
    if (!target || !target->getBody()) return;

    glm::vec3 focus = getFocusPoint();
    glm::vec3 lookDir = getLookDirection();

    if (shiftLock && distance > FIRST_PERSON_CUTOFF) {
        glm::vec3 right(std::cos(heading), 0, -std::sin(heading));
        focus += right * MOUSE_LOCK_OFFSET;
    }

    glm::vec3 desiredCamPos = focus - lookDir * distance;
    glm::vec3 actualCamPos = desiredCamPos;

    if (distance > 0.5f && OWWorld::Active) {
        std::vector<const OWSolver::Body*> ignore;
        OWSolver::Body* rootBody = target->getBody()->getRoot();
        if (rootBody) {
            std::function<void(const OWSolver::Body*)> collect = [&](const OWSolver::Body* b) {
                ignore.push_back(b);
                for (auto* c : b->getChildren()) collect(c);
            };
            collect(rootBody);
        }

        OWWorld::RayHit hit;
        if (OWWorld::Active->raycastClosest(focus, desiredCamPos, hit, ignore)) {
            actualCamPos = hit.point + hit.normal * 0.5f;
        }
    }

    cam.position = toRaylib(actualCamPos);
    cam.target = toRaylib(focus);
    cam.up = {0, 1, 0};
}

float OWCamera::UpdateFirstPersonTransparency(float distance, float dt) {
    constexpr float MAX_TWEEN_RATE = 2.8f;
    
    float transparency = 0.0f;
    bool instant = false;
    
    transparency = (7.0f - distance) / 5.0f;
    if (transparency < 0.5f) {
        transparency = 0.0f;
    }
    
    if (lastTransparency >= 0.0f) {
        float deltaTransparency = transparency - lastTransparency;
        
        if (!instant && transparency < 1.0f && lastTransparency < 0.95f) {
            float maxDelta = MAX_TWEEN_RATE * dt;
            if (deltaTransparency > maxDelta) {
                deltaTransparency = maxDelta;
            }
            if (deltaTransparency < -maxDelta) {
                deltaTransparency = -maxDelta;
            }
        }
        transparency = lastTransparency + deltaTransparency;
    } else {
        transparencyDirty = true;
    }
    
    transparency = std::round(transparency * 100.0f) / 100.0f;
    if (transparency < 0.0f) transparency = 0.0f;
    if (transparency > 1.0f) transparency = 1.0f;
    
    if (transparencyDirty || lastTransparency != transparency) {
        transparencyDirty = false;
        lastTransparency = transparency;
    }
    
    return transparency;
}