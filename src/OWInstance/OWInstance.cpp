#include <algorithm>

#include "OWInstance/OWInstance.hpp"
#include "OWInstance/OWContainer.hpp"

OWInstance::OWInstance() {
    OWContainer::ContainerInstances.push_back(this);
}

OWInstance::~OWInstance() {
    // remove from global container
    auto& container = OWContainer::ContainerInstances;
    container.erase(
        std::remove(container.begin(), container.end(), this),
        container.end()
    );
}