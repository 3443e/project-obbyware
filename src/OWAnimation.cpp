#include "OWAnimation.hpp"
#include "tinyxml2.h"
#include <algorithm>

using namespace tinyxml2;

static OWSolver::CoordinateFrame parseCFrame(const XMLElement* cfElem) {
    float x = cfElem->FirstChildElement("X")->FloatText();
    float y = cfElem->FirstChildElement("Y")->FloatText();
    float z = cfElem->FirstChildElement("Z")->FloatText();
    float r00 = cfElem->FirstChildElement("R00")->FloatText();
    float r01 = cfElem->FirstChildElement("R01")->FloatText();
    float r02 = cfElem->FirstChildElement("R02")->FloatText();
    float r10 = cfElem->FirstChildElement("R10")->FloatText();
    float r11 = cfElem->FirstChildElement("R11")->FloatText();
    float r12 = cfElem->FirstChildElement("R12")->FloatText();
    float r20 = cfElem->FirstChildElement("R20")->FloatText();
    float r21 = cfElem->FirstChildElement("R21")->FloatText();
    float r22 = cfElem->FirstChildElement("R22")->FloatText();

    glm::mat3 rot(
        r00, r10, r20,
        r01, r11, r21,
        r02, r12, r22
    );
    return OWSolver::CoordinateFrame(rot, glm::vec3(x, y, z));
}

static void collectPoses(const XMLElement* elem, Keyframe& kf) {
    while (elem) {
        const char* cls = elem->Attribute("class");
        if (cls && std::string(cls) == "Pose") {
            const XMLElement* props = elem->FirstChildElement("Properties");
            if (props) {
                KeyframePose pose;
                const XMLElement* nameElem = props->FirstChildElement("string");
                while (nameElem) {
                    if (std::string(nameElem->Attribute("name")) == "Name") {
                        pose.name = nameElem->GetText() ? nameElem->GetText() : "";
                        break;
                    }
                    nameElem = nameElem->NextSiblingElement("string");
                }
                const XMLElement* cfElem = props->FirstChildElement("CoordinateFrame");
                if (cfElem) pose.cframe = parseCFrame(cfElem);
                kf.poses.push_back(pose);
            }
        }
        collectPoses(elem->FirstChildElement("Item"), kf);
        elem = elem->NextSiblingElement("Item");
    }
}
void OWAnimation::loadFromFile(const std::string& filename) {
    XMLDocument doc;
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) return;

    const XMLElement* root = doc.RootElement();
    const XMLElement* ksElem = root->FirstChildElement("Item");
    while (ksElem && std::string(ksElem->Attribute("class")) != "KeyframeSequence") {
        ksElem = ksElem->NextSiblingElement("Item");
    }
    if (!ksElem) return;

    const XMLElement* ksProps = ksElem->FirstChildElement("Properties");
    if (ksProps) {
        const XMLElement* loopElem = ksProps->FirstChildElement("bool");
        while (loopElem) {
            if (std::string(loopElem->Attribute("name")) == "Loop") {
                loop = (std::string(loopElem->GetText()) == "true");
                break;
            }
            loopElem = loopElem->NextSiblingElement("bool");
        }
    }

    const XMLElement* kfElem = ksElem->FirstChildElement("Item");
    while (kfElem) {
        const char* cls = kfElem->Attribute("class");
        if (cls && std::string(cls) == "Keyframe") {
            Keyframe kf;
            const XMLElement* props = kfElem->FirstChildElement("Properties");
            if (props) {
                const XMLElement* timeElem = props->FirstChildElement("float");
                while (timeElem) {
                    if (std::string(timeElem->Attribute("name")) == "Time") {
                        kf.time = timeElem->FloatText();
                        break;
                    }
                    timeElem = timeElem->NextSiblingElement("float");
                }
            }

            const XMLElement* poseElem = kfElem->FirstChildElement("Item");
            collectPoses(poseElem, kf);

            keyframes.push_back(kf);
            duration = std::max(duration, kf.time);
        }
        kfElem = kfElem->NextSiblingElement("Item");
    }
}