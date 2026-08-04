#include "../include/physics/world.h"
#include <cstdio>

static void printBody(size_t i, const RigidBody& b) {
    const char* kind = b.shape.type == ShapeType::Sphere ? "sphere" : "box   ";
    printf("  %s %zu: pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f)\n",
           kind, i, b.position.x, b.position.y, b.position.z,
           b.linearVelocity.x, b.linearVelocity.y, b.linearVelocity.z);
}

int main() {
    World world;

    // A few spheres dropped from different heights (bounce + settle)
    world.bodies.push_back(RigidBody::makeDynamicSphere({0.0f, 5.0f, 0.0f}, 0.5f, 1.0f));
    world.bodies.push_back(RigidBody::makeDynamicSphere({0.2f, 8.0f, 0.1f}, 0.5f, 1.0f));

    // A stack of boxes dropped slightly offset (tests box-box SAT + friction)
    world.bodies.push_back(RigidBody::makeDynamicBox({3.0f, 2.0f, 0.0f}, {0.5f, 0.5f, 0.5f}, 1.0f));
    world.bodies.push_back(RigidBody::makeDynamicBox({3.05f, 3.2f, 0.02f}, {0.5f, 0.5f, 0.5f}, 1.0f));
    world.bodies.push_back(RigidBody::makeDynamicBox({2.98f, 4.4f, -0.03f}, {0.5f, 0.5f, 0.5f}, 1.0f));

    // A sphere dropped onto a box (tests sphere-box contact)
    world.bodies.push_back(RigidBody::makeDynamicSphere({3.0f, 6.0f, 0.0f}, 0.4f, 0.5f));

    const float dt = 1.0f / 60.0f;
    const int totalFrames = 600; // 10 seconds

    for (int frame = 0; frame < totalFrames; ++frame) {
        world.step(dt);

        if (frame % 30 == 0) { // print twice a second
            printf("t=%.2fs\n", frame * dt);
            for (size_t i = 0; i < world.bodies.size(); ++i)
                printBody(i, world.bodies[i]);
        }
    }
    return 0;
}
