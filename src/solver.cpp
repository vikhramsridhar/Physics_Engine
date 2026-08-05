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

// The correct "effective mass" denominator for an impulse along `dir` at
// `worldPoint` is NOT just invMassA + invMassB -- that's only correct for
// contacts through the center of mass (true for spheres, since r=0 there).
// For an off-center point (any box corner!), resisting motion along `dir`
// also has to fight rotational inertia, which adds this extra term per body:
//     dir . ( (I^-1 * (r x dir)) x r )
// Skipping this term (an earlier version of this file did) makes the solver
// compute impulses that are too large for off-center contacts, since it
// underestimates how much motion the body actually resists -- the impulse
// overshoots, and the box spins/explodes instead of settling. This was the
// root cause of the box-stacking instability documented in the README.
static float effectiveMassDenominator(const RigidBody& a, const RigidBody& b,
                                       const Vec3& point, const Vec3& dir) {
    float denom = a.invMass + b.invMass;

    if (!a.isStatic) {
        Vec3 ra = point - a.position;
        Vec3 termA = a.applyWorldInvInertia(ra.cross(dir)).cross(ra);
        denom += dir.dot(termA);
    }
    if (!b.isStatic) {
        Vec3 rb = point - b.position;
        Vec3 termB = b.applyWorldInvInertia(rb.cross(dir)).cross(rb);
        denom += dir.dot(termB);
    }
    return denom;
}

void resolveContact(Contact& c, float restitution, float friction) {
    Vec3 relVel = velocityAtPoint(*c.b, c.point) - velocityAtPoint(*c.a, c.point);
    float velAlongNormal = relVel.dot(c.normal);
    if (velAlongNormal > 0.0f) return; // separating already

    float invMassSum = effectiveMassDenominator(*c.a, *c.b, c.point, c.normal);
    if (invMassSum <= 1e-8f) return;

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
        float tangentDenom = effectiveMassDenominator(*c.a, *c.b, c.point, tangent);
        if (tangentDenom > 1e-8f) {
            float jt = -relVel.dot(tangent) / tangentDenom;
            float maxFriction = friction * c.accumNormalImpulse;
            float newTangentImpulse = std::clamp(c.accumTangentImpulse + jt, -maxFriction, maxFriction);
            float deltaTangent = newTangentImpulse - c.accumTangentImpulse;
            c.accumTangentImpulse = newTangentImpulse;

            Vec3 frictionImpulse = tangent * deltaTangent;
            c.a->applyImpulseAtPoint(-frictionImpulse, c.point);
            c.b->applyImpulseAtPoint(frictionImpulse, c.point);
        }
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
