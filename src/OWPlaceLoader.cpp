#include "OWPlaceLoader.hpp"
#include "tinyxml2.h"
#include <iostream>
#include <functional>

using namespace tinyxml2;

static OWSolver::CoordinateFrame ParseCFrame(const XMLElement* cfElem) {
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

static Vector3 ParseVector3(const XMLElement* vecElem) {
    return Vector3{
        vecElem->FirstChildElement("X")->FloatText(),
        vecElem->FirstChildElement("Y")->FloatText(),
        vecElem->FirstChildElement("Z")->FloatText()
    };
}

static Color ParseColor3uint8(unsigned int color) {
    Color c;
    c.r = (color >> 16) & 0xFF;
    c.g = (color >> 8) & 0xFF;
    c.b = color & 0xFF;
    c.a = 255;
    return c;
}

// this finds a token value by name
static int GetToken(const XMLElement* props, const std::string& name) {
    const XMLElement* elem = props->FirstChildElement("token");
    while (elem) {
        if (elem->Attribute("name") && std::string(elem->Attribute("name")) == name) {
            return elem->IntText();
        }
        elem = elem->NextSiblingElement("token");
    }
    return 0;
}

std::vector<OWPart*> OWPlaceLoader::LoadPlace(const std::string& filename, OWWorld& world) {
    std::vector<OWPart*> parts;
    XMLDocument doc;
    
    if (doc.LoadFile(filename.c_str()) != XML_SUCCESS) {
        std::cerr << "Failed to load place file: " << filename << std::endl;
        return parts;
    }

    const XMLElement* root = doc.RootElement();
    if (!root) return parts;

    std::function<void(const XMLElement*)> searchItems = [&](const XMLElement* elem) {
        while (elem) {
            const char* cls = elem->Attribute("class");
            if (cls && std::string(cls) == "Part") {
                const XMLElement* props = elem->FirstChildElement("Properties");
                if (props) {
                    OWPart* part = new OWPart();
                    
                    // part size
                    const XMLElement* sizeElem = props->FirstChildElement("Vector3");
                    while (sizeElem) {
                        if (sizeElem->Attribute("name") && std::string(sizeElem->Attribute("name")) == "size") {
                            part->SetSize(ParseVector3(sizeElem));
                            break;
                        }
                        sizeElem = sizeElem->NextSiblingElement("Vector3");
                    }

                    // part cframe and stuff
                    const XMLElement* cfElem = props->FirstChildElement("CoordinateFrame");
                    while (cfElem) {
                        if (cfElem->Attribute("name") && std::string(cfElem->Attribute("name")) == "CFrame") {
                            OWSolver::CoordinateFrame cf = ParseCFrame(cfElem);
                            part->SetPosition({cf.translation.x, cf.translation.y, cf.translation.z});
                            OWSolver::CoordinateFrame currentCf = part->getBody()->getWorldCFrame();
                            currentCf.rotation = cf.rotation;
                            part->getBody()->setWorldCFrame(currentCf);
                            break;
                        }
                        cfElem = cfElem->NextSiblingElement("CoordinateFrame");
                    }

                    // shape, 0 is ball 1 is block 2 cylinder 3 wedge 4 corner wedge
                    int shapeType = GetToken(props, "shape");
                    switch(shapeType) {
                        case 0:
                            part->SetShapeBall();
                            break;
                        case 2:
                            part->SetShapeCylinder();
                            break;
                        case 3:
                            part->SetShapeWedge();
                            break;
                        case 4:
                            part->SetShapeCornerWedge();
                            break;
                        default:
                            break;
                    }

                    // anchored or not
                    const XMLElement* boolElem = props->FirstChildElement("bool");
                    while (boolElem) {
                        if (boolElem->Attribute("name") && std::string(boolElem->Attribute("name")) == "Anchored") {
                            part->SetAnchored(std::string(boolElem->GetText()) == "true");
                            break;
                        }
                        boolElem = boolElem->NextSiblingElement("bool");
                    }

                    part->SetAnchored(true); // set everything to anchored for now

                    // color stuff
                    const XMLElement* colorElem = props->FirstChildElement("Color3uint8");
                    if (colorElem) {
                        unsigned int colorVal = std::stoul(colorElem->GetText(), nullptr, 10);
                        part->SetColor(ParseColor3uint8(colorVal));
                    } else {
                        part->SetColor(GRAY);
                    }

                    // part name
                    const XMLElement* nameElem = props->FirstChildElement("string");
                    while (nameElem) {
                        if (nameElem->Attribute("name") && std::string(nameElem->Attribute("name")) == "Name") {
                            part->InstanceName = nameElem->GetText() ? nameElem->GetText() : "Part";
                            break;
                        }
                        nameElem = nameElem->NextSiblingElement("string");
                    }

                    // surface

                    int topSurf = GetToken(props, "TopSurface");
                    int botSurf = GetToken(props, "BottomSurface");
                    int frontSurf = GetToken(props, "FrontSurface");
                    int backSurf = GetToken(props, "BackSurface");
                    int leftSurf = GetToken(props, "LeftSurface");
                    int rightSurf = GetToken(props, "RightSurface");

                    if (topSurf == 1 || topSurf == 3 || 
                        botSurf == 1 || botSurf == 3 || 
                        frontSurf == 1 || frontSurf == 3 || 
                        backSurf == 1 || backSurf == 3 || 
                        leftSurf == 1 || leftSurf == 3 || 
                        rightSurf == 1 || rightSurf == 3) {
                        part->SetStudded(true);
                    }

                    parts.push_back(part);
                }
            }
            searchItems(elem->FirstChildElement("Item"));
            elem = elem->NextSiblingElement("Item");
        }
    };

    searchItems(root->FirstChildElement("Item"));

    std::cout << "Loaded " << parts.size() << " parts from " << filename << std::endl;
    return parts;
}