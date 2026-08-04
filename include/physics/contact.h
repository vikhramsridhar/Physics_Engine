#pragma once
#include "rigidbody.h"

struct Contact {
    RigidBody* a = nullptr;
    RigidBody* b = nullptr;
    Vec3 normal;               // points from a to b
    Vec3 point;                 // world-space contact point
    float penetration = 0.0f;
    float accumNormalImpulse = 0.0f;
    float accumTangentImpulse = 0.0f;
};
