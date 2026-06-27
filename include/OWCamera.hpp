#pragma once
#include "OWInstance/OWPart.hpp"
#include <raylib.h>
#include <glm/glm.hpp>

class OWCamera {
public:
    OWCamera();

    void setTarget(OWPart* part) { target = part; }
    void updateInput(float dt);
    void updatePosition(); 

    void setShiftLock(bool enabled);

    bool isShiftLock() const { return shiftLock; }
    void toggleShiftLock();

    float getHeading() const { return heading; }
    float getPitch() const { return pitch; }
    float getDistance() const { return distance; }
    bool isFirstPerson() const { return ( distance < FIRST_PERSON_CUTOFF ); }
    bool isCharacterFacingCamera() const {
        return (shiftLock || isFirstPerson());
    }

    float UpdateFirstPersonTransparency(float distance, float dt);

    glm::vec3 getLookDirection() const;
    glm::vec3 getLookHorizontal() const;

    const Camera& getCamera() const { return cam; }

    static constexpr float DISTANCE_DEFAULT = 12.5f;
    static constexpr float DISTANCE_MIN = 0.5f;
    static constexpr float DISTANCE_MAX = 128.0f;
    static constexpr float FIRST_PERSON_CUTOFF = 1.5f;
    static constexpr float MOUSE_LOCK_OFFSET = 1.5f;
    static constexpr float PITCH_MIN = -1.4f;
    static constexpr float PITCH_MAX = 1.4f;
    static constexpr float MOUSE_SENSITIVITY = 0.003f;
    static constexpr float ZOOM_SPEED = 1.85f;

private:
    OWPart* target = nullptr;
    float heading = 0.0f;
    float pitch = -0.3f;
    float distance = DISTANCE_DEFAULT;
    bool shiftLock = false;
    bool rightMouseDown = false;
    Camera cam = {};

    glm::vec3 getFocusPoint() const;

    float lastTransparency = 0.0f;
    float lastTransparencyUpdate = 0.0f;
    bool transparencyDirty = true;
};