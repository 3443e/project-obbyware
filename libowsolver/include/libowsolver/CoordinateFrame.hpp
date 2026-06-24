#pragma once
#include <glm/glm.hpp>
#include <cmath>

namespace OWSolver {

    struct CoordinateFrame {
        glm::mat3 rotation;
        glm::vec3 translation;

        CoordinateFrame() : rotation(1.0f), translation(0.0f) {}
        CoordinateFrame(const glm::mat3& r, const glm::vec3& t) : rotation(r), translation(t) {}
        explicit CoordinateFrame(const glm::vec3& t) : rotation(1.0f), translation(t) {}

        CoordinateFrame operator*(const CoordinateFrame& other) const {
            return CoordinateFrame(rotation * other.rotation, rotation * other.translation + translation);
        }

        CoordinateFrame inverse() const {
            glm::mat3 rInv = glm::transpose(rotation);
            return CoordinateFrame(rInv, -(rInv * translation));
        }

        glm::vec3 pointToWorldSpace(const glm::vec3& p) const {
            return rotation * p + translation;
        }

        glm::vec3 pointToObjectSpace(const glm::vec3& p) const {
            return glm::transpose(rotation) * (p - translation);
        }

        glm::vec3 vectorToWorldSpace(const glm::vec3& v) const {
            return rotation * v;
        }

        glm::vec3 vectorToObjectSpace(const glm::vec3& v) const {
            return glm::transpose(rotation) * v;
        }

        CoordinateFrame toObjectSpace(const CoordinateFrame& other) const {
            return inverse() * other;
        }

        glm::vec3 column(int i) const {
            return rotation[i];
        }
    };

    inline float zAxisAngle(const glm::mat3& r) {
        return std::atan2(r[0][1], r[0][0]);
    }

    inline double radWrap(double angle) {
        const double twoPi = 6.283185307179586476925286766559;
        angle = std::fmod(angle, twoPi);
        if (angle > 3.14159265358979323846) angle -= twoPi;
        if (angle < -3.14159265358979323846) angle += twoPi;
        return angle;
    }

    inline float deltaRotationClose(float to, float from) {
        return static_cast<float>(radWrap(to - from));
    }

} // namespace OWSolver
