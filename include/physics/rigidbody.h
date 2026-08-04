#pragma once
#include "../math/vec3.h"
#include "../math/quat.h"
#include "shape.h"

struct RigidBody {
    Vec3 position;
    Quat orientation = Quat::identity();
    Vec3 linearVelocity;
    Vec3 angularVelocity;

    Shape shape;

    float mass = 1.0f;
    float invMass = 1.0f;

    // Diagonal inertia tensor in LOCAL space (good enough for boxes/spheres,
    // which are symmetric about their local axes) plus its inverse.
    Vec3 invInertiaLocal{1, 1, 1};

    Vec3 forceAccum;
    Vec3 torqueAccum;
    bool isStatic = false;

    static RigidBody makeDynamicSphere(Vec3 pos, float radius, float m);
    static RigidBody makeDynamicBox(Vec3 pos, Vec3 halfExtents, float m);
    static RigidBody makeStaticBox(Vec3 pos, Vec3 halfExtents);

    void applyForce(const Vec3& f) { forceAccum = forceAccum + f; }
    void applyTorque(const Vec3& t) { torqueAccum = torqueAccum + t; }

    // Apply an impulse at a world-space point (used by contact resolution
    // for boxes, where torque from off-center impulses matters)
    void applyImpulseAtPoint(const Vec3& impulse, const Vec3& worldPoint);

    Mat3 worldInvInertiaTensor() const;
    void integrate(float dt);
};
