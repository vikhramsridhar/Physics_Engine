#pragma once
#include "rigidbody.h"
#include <vector>

struct World {
    std::vector<RigidBody> bodies;
    RigidBody groundBody = RigidBody::makeStaticBox({0, 0, 0}, {1000, 0, 1000}); // representational only
    Vec3 gravity = {0, -9.81f, 0};
    float groundY = 0.0f;
    float restitution = 0.4f;
    float friction = 0.3f;
    int solverIterations = 8;

    void step(float dt);
};
