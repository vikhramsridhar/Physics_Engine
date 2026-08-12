#pragma once
#include "../math/vec3.h"
#include "../math/quat.h"

class GLFWwindow;
// A minimal viewer using legacy fixed-function OpenGL (glBegin/glEnd,
// glLoadMatrixf) via GLFW. This is deliberately NOT a modern shader-based
// renderer -- immediate mode is far less code to get a working viewport,
// which is the point of this file. A "real" renderer (VAOs/VBOs/shaders)
// is a reasonable future upgrade once you want more than wireframe/flat
// shaded boxes and spheres.

struct Camera {
    Vec3 eye{6, 5, 10};
    Vec3 target{0, 1, 0};
    Vec3 up{0, 1, 0};
    float fovDegrees = 60.0f;
    float nearPlane = 0.1f;
    float farPlane = 200.0f;
};

namespace Renderer {

// Returns false if window/context creation failed.
bool init(int width, int height, const char* title);

bool windowShouldClose();

// Call once per frame, before drawing anything.
void beginFrame(const Camera& cam, int framebufferWidth, int framebufferHeight);

// Draws a unit-half-extent box scaled by halfExtents, at position/orientation.
void drawBox(const Vec3& position, const Quat& orientation, const Vec3& halfExtents,
             float r, float g, float b);

// Draws a UV sphere of the given radius at position/orientation.
void drawSphere(const Vec3& position, const Quat& orientation, float radius,
                 float r, float g, float b);

// Simple ground grid on the XZ plane at y = groundY.
void drawGroundGrid(float groundY, float halfSize, int divisions);

// Swaps buffers and polls window/input events. Call once per frame, last.
void endFrame();

void mouse_callback(GLFWwindow* window, double xpos, double ypos);

void shutdown();

} // namespace Renderer
