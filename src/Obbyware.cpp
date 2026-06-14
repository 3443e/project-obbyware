#include <raylib.h>
#include <raymath.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include "OWPhysics/OWPhysics.hpp"
#include "OWPhysics/OWCharacter.hpp"

int main() {
    OWPhysics::Physics Physics;

    Physics.World->getPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

    // Floor
    Physics.AddBox(100, 2, 100, 0, -1, 0);

    OWPhysics::OWCharacter Character(Physics.World, 0, 5, 0); /*  x, y, z */

    InitWindow(800, 450, "obbyware");
    SetTargetFPS(60);

    Camera3D Camera = {};
    Camera.position  = { 0.0f, 8.0f, 16.0f };
    Camera.target = { 0.0f, 0.0f, 0.0f };
    Camera.up = { 0.0f, 1.0f, 0.0f };
    Camera.fovy = 60.0f;
    Camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        float Dt = GetFrameTime();

        // Input
        Vector3 MoveDir = { 0, 0, 0 };
        if (IsKeyDown(KEY_W)) MoveDir.z -= 1.0f;
        if (IsKeyDown(KEY_S)) MoveDir.z += 1.0f;
        if (IsKeyDown(KEY_A)) MoveDir.x -= 1.0f;
        if (IsKeyDown(KEY_D)) MoveDir.x += 1.0f;
        if (IsKeyPressed(KEY_SPACE)) Character.JumpRequested = true;

        if (Vector3Length(MoveDir) > 0) {
            MoveDir = Vector3Normalize(MoveDir);
        }

        Physics.Step(Dt);
        Character.Update(Dt, MoveDir);

        Vector3 Pos = Character.GetPosition();

        // Camera follows character
        Camera.target = Pos;
        Camera.position = { Pos.x, Pos.y + 8.0f, Pos.z + 16.0f };

        BeginDrawing();
            ClearBackground(RAYWHITE);
            BeginMode3D(Camera);
                for (int i = 0; i < 6; i++) {
                    Vector3 LimbPos = Character.GetLimbPosition(i);
                    Vector3 LimbSize = Character.Limbs[i].Size;
                    DrawCube(LimbPos, LimbSize.x, LimbSize.y, LimbSize.z, BLUE);
                    DrawCubeWires(LimbPos, LimbSize.x, LimbSize.y, LimbSize.z, DARKBLUE);
                }
                DrawCube({ 0, -1, 0 }, 100, 2, 100, GRAY);
            EndMode3D();
            DrawText(TextFormat("Y: %.2f", Pos.y), 10, 10, 20, BLACK);
            DrawText(TextFormat("State: %d", (int)Character.State), 10, 30, 20, BLACK);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}