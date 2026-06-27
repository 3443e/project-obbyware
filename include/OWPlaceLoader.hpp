#pragma once
#include "OWWorld.hpp"
#include "OWInstance/OWPart.hpp"
#include <vector>
#include <string>

class OWPlaceLoader {
public:
    static std::vector<OWPart*> LoadPlace(const std::string& filename, OWWorld& world);
};