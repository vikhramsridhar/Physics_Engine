#include "physics/world.h"
#include "render/renderer.h"
#include <GLFW/glfw3.h>

int main() {
    if (!Renderer::init(1024, 768, "Physics Sim Viewer")) {
        return 1;
    }

    World world;
    world.bodies.push_back(RigidBody::makeDynamicSphere({0.0f, 5.0f, 0.0f}, 0.5f, 1.0f));
    world.bodies.push_back(RigidBody::makeDynamicSphere({0.2f, 8.0f, 0.1f}, 0.5f, 1.0f));
    world.bodies.push_back(RigidBody::makeDynamicBox({3.0f, 2.0f, 0.0f}, {0.5f, 0.5f, 0.5f}, 1.0f));
    world.bodies.push_back(RigidBody::makeDynamicBox({3.05f, 3.2f, 0.02f}, {0.5f, 0.5f, 0.5f}, 1.0f));
    world.bodies.push_back(RigidBody::makeDynamicSphere({3.0f, 6.0f, 0.0f}, 0.4f, 0.5f));

    Camera cam;
    cam.eye = {8, 6, 12};
    cam.target = {1.5f, 1.0f, 0};

    const float dt = 1.0f / 60.0f;

    while (!Renderer::windowShouldClose()) {
        world.step(dt);

        int fbW, fbH;
        glfwGetFramebufferSize(glfwGetCurrentContext(), &fbW, &fbH);

        Renderer::beginFrame(cam, fbW, fbH);
        Renderer::drawGroundGrid(world.groundY, 15.0f, 30);

        for (auto& b : world.bodies) {
            if (b.shape.type == ShapeType::Sphere) {
                Renderer::drawSphere(b.position, b.orientation, b.shape.radius, 0.9f, 0.55f, 0.2f);
            } else {
                Renderer::drawBox(b.position, b.orientation, b.shape.halfExtents, 0.3f, 0.6f, 0.9f);
            }
        }
        Renderer::endFrame();
    }

    Renderer::shutdown();
    return 0;
}
