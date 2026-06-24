#include "OWCamera.hpp"
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

    if (shiftLock && !isFirstPerson()) {
        glm::vec3 right(std::cos(heading), 0, -std::sin(heading));
        focus += right * MOUSE_LOCK_OFFSET;
    }

    glm::vec3 camPos;
    if (isFirstPerson()) {
        camPos = focus;
        cam.target = toRaylib(focus + lookDir);
    } else {
        camPos = focus - lookDir * distance;
        cam.target = toRaylib(focus);
    }

    cam.position = toRaylib(camPos);
    cam.up = {0, 1, 0};
}