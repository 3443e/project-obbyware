#pragma once

#include <glm/ext/vector_float3.hpp>
#include <glm/geometric.hpp>

namespace OWLighting {
    inline glm::vec3 sunDir = glm::normalize(glm::vec3(-0.45f, -0.75f, -0.35f));
    inline glm::vec3 sunColor = glm::vec3(1.00f, 0.97f, 0.88f) * 0.85f;
    inline glm::vec3 skyUp = glm::vec3(0.45f, 0.65f, 0.95f);
    inline glm::vec3 skyDown = glm::vec3(0.55f, 0.55f, 0.50f);
    inline glm::vec3 ambient = glm::vec3(0.2f);    
}