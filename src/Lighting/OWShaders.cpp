#include "Lighting/OWShaders.hpp"
#include "Lighting/OWLighting.hpp"
#include "OWCamera.hpp"

Shader OWShaders::lightingShader;
Camera3D* OWShaders::currentCamera = nullptr;
Texture2D OWShaders::studTexture;
Texture2D OWShaders::grayTexture;

void OWShaders::InitStudTexture() {
    studTexture = LoadTexture("assets/stud5.png");
    SetTextureWrap(studTexture, TEXTURE_WRAP_REPEAT);
    SetTextureFilter(studTexture, TEXTURE_FILTER_BILINEAR);
}

void OWShaders::GenerateGrayTexture() {
    Image img = GenImageColor(1, 1, (Color){128, 128, 128, 0});
    grayTexture = LoadTextureFromImage(img);
    UnloadImage(img);
}

void OWShaders::InitOWShaders() {
    lightingShader = LoadShader("assets/shaders/lighting.vs", "assets/shaders/lighting.fs");

    OWShaders::InitStudTexture();
    OWShaders::GenerateGrayTexture();

    int studSlot = 0;  // texture unit 0
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "studMap"), &studSlot, SHADER_UNIFORM_SAMPLER2D);

    float studScale = 0.5f;
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "studScale"), &studScale, SHADER_UNIFORM_FLOAT);
}

void OWShaders::OWBeginLighting(Camera3D& cam) {
    glm::vec3 camPos = {cam.position.x, cam.position.y, cam.position.z};
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "cameraPos"), &camPos, SHADER_UNIFORM_VEC3);
    BeginShaderMode(lightingShader);
}

void OWShaders::OWEndLighting() {
    EndShaderMode();
}

void OWShaders::OWUpdateLightingShaderValues() {
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "lightDir"), &OWLighting::sunDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "lightColor"), &OWLighting::sunColor, SHADER_UNIFORM_VEC3);
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "ambientUp"), &OWLighting::skyUp, SHADER_UNIFORM_VEC3);
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "ambientDown"), &OWLighting::skyDown, SHADER_UNIFORM_VEC3);
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "ambientColor"), &OWLighting::ambient, SHADER_UNIFORM_VEC3);
    SetShaderValue(lightingShader, GetShaderLocation(lightingShader, "fogColor"), &OWLighting::fogColor, SHADER_UNIFORM_VEC3);
}

