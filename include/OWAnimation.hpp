#pragma once
#include "libowsolver/CoordinateFrame.hpp"
#include <string>
#include <vector>

struct KeyframePose {
    std::string name;
    OWSolver::CoordinateFrame cframe;
};

struct Keyframe {
    float time;
    std::vector<KeyframePose> poses;
};

struct OWAnimation {
    std::string name;
    bool loop = true;
    std::vector<Keyframe> keyframes;
    float duration = 0.0f;

    void loadFromFile(const std::string& filename);
};