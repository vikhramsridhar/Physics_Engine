#include "../include/physics/solver.h"
#include <algorithm>

// Relative velocity at the contact point, accounting for angular velocity
// (v_point = v_linear + omega x r). Needed for correct box behavior, since
// contact points aren't at the center of mass the way they are for spheres.
static Vec3 velocityAtPoint(const RigidBody& body, const Vec3& worldPoint) {
    if (body.isStatic) return {};
    Vec3 r = worldPoint - body.position;
    return body.linearVelocity + body.angularVelocity.cross(r);
}

void resolveContact(Contact& c, float restitution, float friction) {
    Vec3 relVel = velocityAtPoint(*c.b, c.point) - velocityAtPoint(*c.a, c.point);
    float velAlongNormal = relVel.dot(c.normal);
    if (velAlongNormal > 0.0f) return; // separating already

    float invMassSum = c.a->invMass + c.b->invMass;
    if (invMassSum <= 0.0f) return;

    // Normal impulse
    float j = -(1.0f + restitution) * velAlongNormal / invMassSum;
    float newImpulse = std::max(0.0f, c.accumNormalImpulse + j);
    float deltaImpulse = newImpulse - c.accumNormalImpulse;
    c.accumNormalImpulse = newImpulse;

    Vec3 impulse = c.normal * deltaImpulse;
    c.a->applyImpulseAtPoint(-impulse, c.point);
    c.b->applyImpulseAtPoint(impulse, c.point);

    // Friction impulse (Coulomb, tangent to normal)
    relVel = velocityAtPoint(*c.b, c.point) - velocityAtPoint(*c.a, c.point);
    Vec3 tangent = relVel - c.normal * relVel.dot(c.normal);
    float tangentLen = tangent.length();
    if (tangentLen > 1e-6f) {
        tangent = tangent * (1.0f / tangentLen);
        float jt = -relVel.dot(tangent) / invMassSum;
        float maxFriction = friction * c.accumNormalImpulse;
        float newTangentImpulse = std::clamp(c.accumTangentImpulse + jt, -maxFriction, maxFriction);
        float deltaTangent = newTangentImpulse - c.accumTangentImpulse;
        c.accumTangentImpulse = newTangentImpulse;

        Vec3 frictionImpulse = tangent * deltaTangent;
        c.a->applyImpulseAtPoint(-frictionImpulse, c.point);
        c.b->applyImpulseAtPoint(frictionImpulse, c.point);
    }
}

void positionalCorrection(Contact& c) {
    const float percent = 0.8f;
    const float slop = 0.01f;
    float invMassSum = c.a->invMass + c.b->invMass;
    if (invMassSum <= 0.0f) return;

    float corrMag = std::max(c.penetration - slop, 0.0f) / invMassSum * percent;
    Vec3 correction = c.normal * corrMag;
    c.a->position = c.a->position - correction * c.a->invMass;
    c.b->position = c.b->position + correction * c.b->invMass;
}

void solveContacts(std::vector<Contact>& contacts, float restitution,
                    float friction, int iterations) {
    for (int iter = 0; iter < iterations; ++iter)
        for (auto& c : contacts)
            resolveContact(c, restitution, friction);

    for (auto& c : contacts)
        positionalCorrection(c);
}
