#include <raylib.h>
#include <rlgl.h>
#include <glm/glm.hpp>

inline Shader g_lightingShader;
inline Camera3D* g_currentCamera = nullptr;
inline Texture2D g_studTexture;

inline void InitStudTexture() {
    g_studTexture = LoadTexture("assets/stud5.png");
    SetTextureWrap(g_studTexture, TEXTURE_WRAP_REPEAT);
    SetTextureFilter(g_studTexture, TEXTURE_FILTER_BILINEAR);
}
inline Texture2D g_grayTexture;

inline void InitGrayTexture() {
    Image img = GenImageColor(1, 1, (Color){128, 128, 128, 0});
    g_grayTexture = LoadTextureFromImage(img);
    UnloadImage(img);
}

inline void InitLighting() {
    g_lightingShader = LoadShader("assets/shaders/lighting.vs", "assets/shaders/lighting.fs");

    InitStudTexture();
    InitGrayTexture();
    glm::vec3 sunDir = glm::normalize(glm::vec3(-0.45f, -0.75f, -0.35f));
    glm::vec3 sunColor = glm::vec3(1.00f, 0.97f, 0.88f) * 0.85f;
    glm::vec3 skyUp = glm::vec3(0.45f, 0.65f, 0.95f);
    glm::vec3 skyDown = glm::vec3(0.55f, 0.55f, 0.50f);
    glm::vec3 ambient = glm::vec3(0.2f);    
    glm::vec3 fogColor = glm::vec3(0.75f, 0.82f, 0.92f); 

    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "lightDir"), &sunDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "lightColor"), &sunColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "ambientUp"), &skyUp, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "ambientDown"), &skyDown, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "ambientColor"), &ambient, SHADER_UNIFORM_VEC3);
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "fogColor"), &fogColor, SHADER_UNIFORM_VEC3);

    int studSlot = 0;  // texture unit 0
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "studMap"), &studSlot, SHADER_UNIFORM_SAMPLER2D);

    float studScale = 0.5f;
    //float studIntensity = 1.0f;
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "studScale"), &studScale, SHADER_UNIFORM_FLOAT);
    //SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "studIntensity"), &studIntensity, SHADER_UNIFORM_FLOAT);
}

inline void BeginLighting(Camera3D& cam) {
    glm::vec3 camPos = {cam.position.x, cam.position.y, cam.position.z};
    SetShaderValue(g_lightingShader, GetShaderLocation(g_lightingShader, "cameraPos"), &camPos, SHADER_UNIFORM_VEC3);
    BeginShaderMode(g_lightingShader);
}

inline void EndLighting() {
    EndShaderMode();
}