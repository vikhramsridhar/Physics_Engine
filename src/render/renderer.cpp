#include "../include/render/renderer.h"

// GLFW pulls in the system GL headers itself when GLFW_INCLUDE_NONE is not
// defined, which is exactly what we want for the legacy (fixed-function)
// GL calls used below (glBegin/glEnd/glLoadMatrixf etc).
#include <GLFW/glfw3.h>
#include <cmath>
#include <vector>
#include <cstdio>
#include <iostream>

namespace {
    GLFWwindow* g_window = nullptr;

    // Column-major 4x4, matching what glLoadMatrixf/glMultMatrixf expect.
    struct Mat4 { float m[16]; };

    Mat4 identity4() {
        Mat4 r{};
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
        return r;
    }

    // Builds a view matrix equivalent to gluLookAt, written by hand so we
    // don't need GLU as a dependency.
    Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
        Vec3 f = (target - eye).normalized();
        Vec3 s = f.cross(up).normalized();
        Vec3 u = s.cross(f);

        Mat4 r = identity4();
        r.m[0] = s.x;  r.m[4] = s.y;  r.m[8]  = s.z;
        r.m[1] = u.x;  r.m[5] = u.y;  r.m[9]  = u.z;
        r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
        r.m[12] = -s.dot(eye);
        r.m[13] = -u.dot(eye);
        r.m[14] =  f.dot(eye);
        return r;
    }

    // Builds a model matrix (translation * rotation) for a body's pose,
    // suitable for glMultMatrixf.
    Mat4 modelMatrix(const Vec3& position, const Quat& orientation) {
        Mat3 rot = orientation.toMat3();
        Mat4 r = identity4();
        r.m[0] = rot.col0.x; r.m[1] = rot.col0.y; r.m[2] = rot.col0.z;
        r.m[4] = rot.col1.x; r.m[5] = rot.col1.y; r.m[6] = rot.col1.z;
        r.m[8] = rot.col2.x; r.m[9] = rot.col2.y; r.m[10] = rot.col2.z;
        r.m[12] = position.x; r.m[13] = position.y; r.m[14] = position.z;
        return r;
    }

    void errorCallback(int code, const char* desc) {
        fprintf(stderr, "GLFW error %d: %s\n", code, desc);
    }
}

namespace Renderer {

bool init(int width, int height, const char* title) {
    glfwSetErrorCallback(errorCallback);
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return false;
    }

    // Request a legacy-compatible context so glBegin/glEnd style calls work.
    // (Core profiles reject fixed-function calls entirely.)
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_ANY_PROFILE);

    g_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!g_window) {
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(g_window);
    glfwSetInputMode(g_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); 
    glfwSetCursorPosCallback(g_window, mouse_callback);
    glfwSwapInterval(1); // vsync

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

    GLfloat lightPos[] = { 5.0f, 10.0f, 5.0f, 1.0f };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    GLfloat ambient[] = { 0.25f, 0.25f, 0.25f, 1.0f };
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);

    glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
    return true;
}

bool windowShouldClose() {
    return g_window == nullptr || glfwWindowShouldClose(g_window);
}

void beginFrame(const Camera& cam, int fbWidth, int fbHeight) {
    glViewport(0, 0, fbWidth, fbHeight);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float aspect = fbHeight > 0 ? (float)fbWidth / (float)fbHeight : 1.0f;
    float fovRad = cam.fovDegrees * 3.14159265f / 180.0f;
    float top = std::tan(fovRad * 0.5f) * cam.nearPlane;
    float right = top * aspect;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-right, right, -top, top, cam.nearPlane, cam.farPlane);

    glMatrixMode(GL_MODELVIEW);
    Mat4 view = lookAt(cam.eye, cam.target, cam.up);
    glLoadMatrixf(view.m);
    Camera* camPtr = const_cast<Camera*>(&cam);
    glfwSetWindowUserPointer(g_window, camPtr);
}

void drawBox(const Vec3& position, const Quat& orientation, const Vec3& he,
             float r, float g, float b) {
    glPushMatrix();
    Mat4 model = modelMatrix(position, orientation);
    glMultMatrixf(model.m);
    glColor3f(r, g, b);

    // 6 faces, each a quad with an outward normal for lighting.
    struct Face { Vec3 normal; Vec3 v[4]; };
    Face faces[6] = {
        { { 1,0,0}, { {he.x,-he.y,-he.z}, {he.x,he.y,-he.z}, {he.x,he.y,he.z}, {he.x,-he.y,he.z} } },
        { {-1,0,0}, { {-he.x,-he.y,he.z}, {-he.x,he.y,he.z}, {-he.x,he.y,-he.z}, {-he.x,-he.y,-he.z} } },
        { {0, 1,0}, { {-he.x,he.y,-he.z}, {-he.x,he.y,he.z}, {he.x,he.y,he.z}, {he.x,he.y,-he.z} } },
        { {0,-1,0}, { {-he.x,-he.y,he.z}, {-he.x,-he.y,-he.z}, {he.x,-he.y,-he.z}, {he.x,-he.y,he.z} } },
        { {0,0, 1}, { {-he.x,-he.y,he.z}, {he.x,-he.y,he.z}, {he.x,he.y,he.z}, {-he.x,he.y,he.z} } },
        { {0,0,-1}, { {he.x,-he.y,-he.z}, {-he.x,-he.y,-he.z}, {-he.x,he.y,-he.z}, {he.x,he.y,-he.z} } },
    };

    glBegin(GL_QUADS);
    for (auto& f : faces) {
        glNormal3f(f.normal.x, f.normal.y, f.normal.z);
        for (auto& v : f.v) glVertex3f(v.x, v.y, v.z);
    }
    glEnd();
    glPopMatrix();
}

void drawSphere(const Vec3& position, const Quat& orientation, float radius,
                 float r, float g, float b) {
    (void)orientation; // a sphere's silhouette doesn't change with rotation
    glPushMatrix();
    glTranslatef(position.x, position.y, position.z);
    glColor3f(r, g, b);

    const int stacks = 12, slices = 16;
    for (int i = 0; i < stacks; ++i) {
        float lat0 = 3.14159265f * (-0.5f + (float)i / stacks);
        float lat1 = 3.14159265f * (-0.5f + (float)(i + 1) / stacks);
        float y0 = std::sin(lat0), y1 = std::sin(lat1);
        float rr0 = std::cos(lat0), rr1 = std::cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; ++j) {
            float lng = 2.0f * 3.14159265f * (float)j / slices;
            float x = std::cos(lng), z = std::sin(lng);

            Vec3 n0{x * rr0, y0, z * rr0};
            Vec3 n1{x * rr1, y1, z * rr1};
            glNormal3f(n0.x, n0.y, n0.z);
            glVertex3f(n0.x * radius, n0.y * radius, n0.z * radius);
            glNormal3f(n1.x, n1.y, n1.z);
            glVertex3f(n1.x * radius, n1.y * radius, n1.z * radius);
        }
        glEnd();
    }
    glPopMatrix();
}

void drawGroundGrid(float groundY, float halfSize, int divisions) {
    glDisable(GL_LIGHTING);
    glColor3f(0.35f, 0.35f, 0.4f);
    glBegin(GL_LINES);
    float step = (halfSize * 2.0f) / divisions;
    for (int i = 0; i <= divisions; ++i) {
        float offset = -halfSize + i * step;
        glVertex3f(offset, groundY, -halfSize);
        glVertex3f(offset, groundY,  halfSize);
        glVertex3f(-halfSize, groundY, offset);
        glVertex3f( halfSize, groundY, offset);
    }
    glEnd();
    glEnable(GL_LIGHTING);
}

void endFrame() {
    glfwSwapBuffers(g_window);
    glfwPollEvents();
}

void shutdown() {
    if (g_window) glfwDestroyWindow(g_window);
    glfwTerminate();
    g_window = nullptr;
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos) {

    Camera* cam = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    
    if (cam != nullptr) {
        cam->eye.x = static_cast<float>(xpos/20);
        cam->eye.y = static_cast<float>(ypos/20);

        // For debugging purposes
        //std::cout << "Mouse position: " << xpos/20 << ", " << ypos/20 << std::endl;
    }
}

} // namespace Renderer
