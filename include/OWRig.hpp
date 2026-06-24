#pragma once
#include "OWInstance/OWPart.hpp"
#include <raylib.h>

class OWRig {
public:
    OWRig();

    void SetPosition(const Vector3& pos);
    Vector3 GetPosition() const;

    OWPart* getRoot() {
        return &rootPart;
    }

    void Render();

    OWPart& getTorso() { return torso; }
    OWPart& getHead() { return head; }
    OWPart& getLeftArm() { return leftArm; }
    OWPart& getRightArm() { return rightArm; }
    OWPart& getLeftLeg() { return leftLeg; }
    OWPart& getRightLeg() { return rightLeg; }

private:
    // the order matters btw the rootPart must be constructed first
    OWPart rootPart; 
    OWPart torso;
    OWPart head;
    OWPart leftArm;
    OWPart rightArm;
    OWPart leftLeg;
    OWPart rightLeg;
};