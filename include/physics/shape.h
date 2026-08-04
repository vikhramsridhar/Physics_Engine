#pragma once
#include "../math/vec3.h"

enum class ShapeType { Sphere, Box };

// A shape carries only geometry data; the RigidBody carries pose/mass/velocity.
struct Shape {
    ShapeType type;

    // Sphere
    float radius = 0.5f;

    // Box (axis-aligned in local/body space; half-extents along each local axis)
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};

    static Shape makeSphere(float r) {
        Shape s; s.type = ShapeType::Sphere; s.radius = r; return s;
    }
    static Shape makeBox(Vec3 halfExtents) {
        Shape s; s.type = ShapeType::Box; s.halfExtents = halfExtents; return s;
    }
};
