#include <raylib.h>
#include <rlgl.h>
#include <glm/glm.hpp>

static Shader g_lightingShader;
static Camera3D* g_currentCamera = nullptr;

inline void InitLighting() {
    g_lightingShader = LoadShader("assets/shaders/lighting.vs", "assets/shaders/lighting.fs");

    glm::vec3 sunDir = glm::normalize(glm::vec3(-0.55f, -0.35f, -0.45f));  // low angle
    //glm::vec3 sunColor   = glm::vec3(1.0f, 0.55f, 0.32f) * 1.0f;            // orange-red
    // Soft sky blue
    glm::vec3 sunColor = glm::vec3(0.45f, 0.70f, 1.00f) * 1.0f;
    glm::vec3 skyUp = glm::vec3(0.32f, 0.28f, 0.42f);      
    glm::vec3 skyDown = glm::vec3(0.20f, 0.13f, 0.10f);   
    glm::vec3 ambient = glm::vec3(0.5f);
    glm::vec3 fogColor = glm::vec3(0.55f, 0.40f, 0.42f);           

    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "lightDir"), &sunDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "lightColor"), &sunColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "ambientUp"), &skyUp, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "ambientDown"), &skyDown, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "ambientColor"), &ambient, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "fogColor"), &fogColor, SHADER_UNIFORM_VEC3);
}

inline void BeginLighting(Camera3D& cam) {
    glm::vec3 camPos = {cam.position.x, cam.position.y, cam.position.z};
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "cameraPos"), &camPos, SHADER_UNIFORM_VEC3);

    BeginShaderMode(g_lightingShader);
}

inline void EndLighting() {
    EndShaderMode();
}