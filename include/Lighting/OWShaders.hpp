#pragma once

#include <raylib.h>
#include <rlgl.h>
#include <glm/glm.hpp>

namespace OWShaders {
    extern Shader lightingShader;
    extern Camera3D* currentCamera;
    extern Texture2D studTexture;
    extern Texture2D grayTexture;

    void InitStudTexture();
    void GenerateGrayTexture();

    void InitOWShaders();
    void OWBeginLighting(Camera3D& cam);
    void OWEndLighting();
    void OWUpdateLightingShaderValues();
}