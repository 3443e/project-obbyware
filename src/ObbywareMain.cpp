#include "ObbywareMain.hpp"
#include "OWDebugFlags.hpp"
#include "OWInstance/OWPart.hpp"
#include "OWInstance/OWContainer.hpp"
#include "OWWorld.hpp"
#include "OWPlayerController.hpp"
#include "OWRig.hpp"
#include "OWCamera.hpp"
#include <raylib.h>
#include <string>
#include <glm/glm.hpp>

int main() {
    OWMain::CurrentMonitorRefreshRate = GetMonitorRefreshRate(GetCurrentMonitor());

    InitWindow(800, 500, "OBBYWARE");
    SetWindowState(FLAG_WINDOW_RESIZABLE);
    SetTargetFPS(240);
    EnableCursor();

    // physics world
    OWWorld world;

    OWPart floor;
    floor.InstanceName = "Floor";
    floor.SetSize({50, 1, 50});
    floor.SetAnchored(true);
    floor.SetPosition({0, -0.5f, 0});
    floor.SetColor(GRAY);

    OWPart floor2;
    floor2.InstanceName = "Floor";
    floor2.SetSize({10, 1, 10});
    floor2.SetAnchored(true);
    floor2.SetPosition({0, 0.5f, 0});
    floor2.SetColor(GRAY);

    OWPart wall1;
    wall1.InstanceName = "Wall1";
    wall1.SetSize({40, 4, 1});
    wall1.SetAnchored(true);
    wall1.SetPosition({0, 2, -10});
    wall1.SetColor(DARKGRAY);

    OWPart wall2;
    wall2.InstanceName = "Wall2";
    wall2.SetSize({2, 4, 40});
    wall2.SetAnchored(true);
    wall2.SetPosition({10, 2, 0});
    wall2.SetColor(DARKGRAY);

    OWPart truss;
    truss.InstanceName = "Truss";
    truss.SetSize({2, 100, 2});
    truss.SetAnchored(true);
    truss.SetPosition({5, 4.5f, 0});
    truss.SetColor(BROWN);
    truss.SetTruss(true);

    /*
    OWPart wedge;
    wedge.InstanceName = "Wedge";
    wedge.SetSize({4, 8, 8});
    wedge.SetAnchored(true);
    wedge.SetPosition({-8, 2, 5});
    wedge.SetColor(GREEN);
    wedge.SetShapeWedge();

    OWPart ball;
    ball.InstanceName = "Ball";
    ball.SetSize({23, 23, 23});
    ball.SetAnchored(true);
    ball.SetPosition({-5, 1.5f, -5});
    ball.SetColor(BLUE);
    ball.SetShapeBall();
    

    OWPart cyl;
    cyl.InstanceName = "Cylinder";
    cyl.SetSize({8, 3, 3});
    cyl.SetAnchored(true);
    cyl.SetPosition({8, 1.5f, -5});
    cyl.SetColor(YELLOW);
    cyl.SetShapeCylinder();
    
    OWPart cornerWedge;
    cornerWedge.InstanceName = "CornerWedge";
    cornerWedge.SetSize({4, 4, 4});
    cornerWedge.SetAnchored(true);
    cornerWedge.SetPosition({15, 2, 5});
    cornerWedge.SetColor(PURPLE);
    cornerWedge.SetShapeCornerWedge();
    */
    // creating a rig for the player
    OWRig character;
    character.SetPosition({0, 5, 0});

    // initializing the camera 
    OWCamera camera;
    camera.setTarget(character.getRoot());

    // initializing the character 
    OWPlayerController controller(character.getRoot());
    controller.setCamera(&camera);
    controller.setJumpPower(50.0f);
    controller.setWalkSpeed(16.0f);
    controller.setHipHeight(0.0f);

    OWAnimation walkAnim;     walkAnim.loadFromFile("assets/animations/WalkLoopAnimation_180426354.rbxmx");
    OWAnimation idleAnim;     idleAnim.loadFromFile("assets/animations/Idle1LoopAnimation_180435571.rbxmx");
    OWAnimation jumpAnim;     jumpAnim.loadFromFile("assets/animations/JumpAnimation_125750702.rbxmx");
    OWAnimation fallAnim;     fallAnim.loadFromFile("assets/animations/FallLoopAnimation_180436148.rbxmx");
    OWAnimation climbAnim;    climbAnim.loadFromFile("assets/animations/ClimbLoopAnimation_180436334.rbxmx");

    // 60Hz game step accumulator (Humanoid::onStepped rate)
    float gameStepAccumulator = 0.0f;
    constexpr float GAME_STEP_DT = 1.0f / 60.0f;

    while (!WindowShouldClose()) {
        if (IsKeyReleased(KEY_F3)) OWDebugFlags::DEBUG_SHOW_INFO = !OWDebugFlags::DEBUG_SHOW_INFO;
        if (IsKeyReleased(KEY_F4)) OWDebugFlags::DEBUG_SHOW_RAYS = !OWDebugFlags::DEBUG_SHOW_RAYS;
        controller.setDebugRaysEnabled(OWDebugFlags::DEBUG_SHOW_RAYS);

        float frameTime = GetFrameTime();

        camera.updateInput(frameTime);

        // game step (fixed 60Hz)
        gameStepAccumulator += frameTime;
        while (gameStepAccumulator >= GAME_STEP_DT) {
            controller.updateInput(GAME_STEP_DT);

            const OWAnimation* target = &idleAnim;
            bool moving = IsKeyDown(KEY_W) || IsKeyDown(KEY_A) || IsKeyDown(KEY_S) || IsKeyDown(KEY_D);

            if (controller.getState() == OWPlayerController::State::Climbing) {
                target = &climbAnim;
            } else if (controller.isJumping()) {
                target = &jumpAnim;
            } else if (!controller.isGrounded()) {
                target = &fallAnim;
            } else if (moving) {
                target = &walkAnim;
            }

            if (character.getCurrentAnim() != target) {
                character.playAnimation(target);
            }
            character.updateAnimation(GAME_STEP_DT);

            gameStepAccumulator -= GAME_STEP_DT;
        }
        // prevent spiral of death if FPS drops below 60
        if (gameStepAccumulator > GAME_STEP_DT * 2.0f) {
            gameStepAccumulator = 0.0f;
        }

        // step physics (240Hz)
        world.step(frameTime);

        // update the camera position (uses character position from physics update)
        camera.updatePosition();
        OWMain::CurrentPlayerCamera = camera.getCamera();

        BeginDrawing();
            ClearBackground({179, 179, 203, 0});

            BeginMode3D(OWMain::CurrentPlayerCamera);
                for (OWInstance* child : OWContainer::ContainerInstances) {
                    child->Render();
                }
                DrawGrid(20, 1.0f);
                controller.renderDebugRays();
            EndMode3D();

            if (OWDebugFlags::DEBUG_SHOW_INFO) {
                DrawRectangle(0, 0, 360, 300, (Color) {0, 0, 0, 160});
                DrawText("obbyware", 10, 8, 12, PURPLE);
                DrawText("----------------------", 10, 24, 10, WHITE);

                int currentFPS = GetFPS();
                DrawText(std::string("fps:  " + std::to_string(currentFPS)).c_str(), 10, 38, 10, currentFPS > 30 ? GREEN : RED);
                DrawText(std::string("dt:   " + std::to_string(GetFrameTime()).substr(0, 6) + "s").c_str(), 10, 50, 10, WHITE);
                DrawText("----------------------", 10, 64, 10, WHITE);

                Vector3 ppos = character.GetPosition();
                DrawText(std::string("pos:  " + std::to_string(ppos.x).substr(0, 6) + ", " + std::to_string(ppos.y).substr(0, 6) + ", " + std::to_string(ppos.z).substr(0, 6)).c_str(), 10, 76, 10, WHITE);
                glm::vec3 vel = character.getRoot()->getBody()->getLinearVelocity();
                DrawText(std::string("vel:  " + std::to_string(vel.x).substr(0, 6) + ", " + std::to_string(vel.y).substr(0, 6) + ", " + std::to_string(vel.z).substr(0, 6)).c_str(), 10, 88, 10, WHITE);
                float horizSpeed = std::sqrt(vel.x * vel.x + vel.z * vel.z);
                DrawText(std::string("hspd: " + std::to_string(horizSpeed).substr(0, 5) + " st/s").c_str(), 10, 100, 10, horizSpeed > 0.5f ? YELLOW : GRAY);
                float mass = character.getRoot()->getBody()->getBranchMass();
                DrawText(std::string("mass: " + std::to_string(mass)).c_str(), 10, 112, 10, WHITE);
                DrawText("----------------------", 10, 126, 10, WHITE);

                Color groundedColor = controller.isGrounded() ? GREEN : RED;
                DrawText(std::string("grounded:    " + std::string(controller.isGrounded() ? "YES" : "no")).c_str(), 10, 138, 10, groundedColor);
                Color jumpingColor = controller.isJumping() ? YELLOW : GRAY;
                DrawText(std::string("jumping:     " + std::string(controller.isJumping() ? "YES" : "no")).c_str(), 10, 150, 10, jumpingColor);
                Color hipColor = controller.isHipEnabled() ? GREEN : GRAY;
                DrawText(std::string("hip ctrl:    " + std::string(controller.isHipEnabled() ? "ON" : "off")).c_str(), 10, 162, 10, hipColor);
                float floorDist = controller.getFloorDist();
                Color floorColor = floorDist < 4.0f ? GREEN : (floorDist < 10.0f ? YELLOW : RED);
                DrawText(std::string("floor dist:  " + std::to_string(floorDist).substr(0, 5) + " st").c_str(), 10, 174, 10, floorColor);
                DrawText(std::string("jump timer:  " + std::to_string(controller.getJumpTimer()).substr(0, 5) + "s").c_str(), 10, 186, 10, controller.isJumping() ? YELLOW : GRAY);
                DrawText(std::string("contacts:    " + std::to_string(world.getContactCount())).c_str(), 10, 198, 10, GREEN);
                DrawText("----------------------", 10, 212, 10, WHITE);

                DrawText(std::string("shift lock:  " + std::string(camera.isShiftLock() ? "on" : "off")).c_str(), 10, 224, 10, camera.isShiftLock() ? YELLOW : GRAY);
                DrawText(std::string("first person:" + std::string(camera.isFirstPerson() ? "yes" : "no")).c_str(), 10, 236, 10, camera.isFirstPerson() ? SKYBLUE : GRAY);
                DrawText(std::string("cam dist:    " + std::to_string(camera.getDistance()).substr(0, 5)).c_str(), 10, 248, 10, WHITE);
                DrawText("----------------------", 10, 262, 10, WHITE);
            }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}