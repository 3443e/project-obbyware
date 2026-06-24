#pragma once
#include <cstddef>
#include <string>
#include <vector>

class OWInstance {
public:
    OWInstance();
    virtual ~OWInstance();

    std::string InstanceName;

    virtual void Render() {}
};