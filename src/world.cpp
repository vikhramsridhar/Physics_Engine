#include "../include/physics/world.h"
#include "../include/physics/collision.h"
#include "../include/physics/solver.h"

void World::step(float dt) {
    // 1. Apply forces
    for (auto& b : bodies)
        if (!b.isStatic) b.applyForce(gravity * b.mass);

    // 2. Integrate
    for (auto& b : bodies) b.integrate(dt);

    // 3. Collect contacts (sphere-sphere, sphere-box, box-box, and vs ground)
    std::vector<Contact> contacts;
    generateContacts(bodies, groundBody, groundY, contacts);

    // 4. Solve velocity constraints + positional correction
    solveContacts(contacts, restitution, friction, solverIterations);
}
