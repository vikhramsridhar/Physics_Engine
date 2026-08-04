#include "../include/physics/rigidbody.h"

RigidBody RigidBody::makeDynamicSphere(Vec3 pos, float radius, float m) {
    RigidBody b;
    b.position = pos;
    b.shape = Shape::makeSphere(radius);
    b.mass = m;
    b.invMass = 1.0f / m;
    // Solid sphere inertia: I = 2/5 * m * r^2 (same about every axis)
    float I = 0.4f * m * radius * radius;
    b.invInertiaLocal = { 1.0f / I, 1.0f / I, 1.0f / I };
    return b;
}

RigidBody RigidBody::makeDynamicBox(Vec3 pos, Vec3 halfExtents, float m) {
    RigidBody b;
    b.position = pos;
    b.shape = Shape::makeBox(halfExtents);
    b.mass = m;
    b.invMass = 1.0f / m;
    // Solid box inertia tensor (diagonal, in local space):
    // Ixx = m/12 * (h^2 + d^2), etc., where h/w/d are FULL extents (2*half)
    float w = halfExtents.x * 2, h = halfExtents.y * 2, d = halfExtents.z * 2;
    float Ixx = (m / 12.0f) * (h*h + d*d);
    float Iyy = (m / 12.0f) * (w*w + d*d);
    float Izz = (m / 12.0f) * (w*w + h*h);
    b.invInertiaLocal = { 1.0f / Ixx, 1.0f / Iyy, 1.0f / Izz };
    return b;
}

RigidBody RigidBody::makeStaticBox(Vec3 pos, Vec3 halfExtents) {
    RigidBody b;
    b.position = pos;
    b.shape = Shape::makeBox(halfExtents);
    b.invMass = 0.0f;
    b.invInertiaLocal = {0, 0, 0};
    b.isStatic = true;
    return b;
}

Mat3 RigidBody::worldInvInertiaTensor() const {
    // I_world^-1 = R * I_local^-1 * R^T  (R orthonormal, from orientation)
    Mat3 R = orientation.toMat3();
    Mat3 result;
    // Scale each local axis (column of R) by the corresponding inverse inertia,
    // then re-express as a matrix: since I_local^-1 is diagonal this simplifies to
    // scaling columns of R by invInertiaLocal, then combining with R^T on apply.
    // We store it as a pair (R, invInertiaLocal) implicitly via this helper below.
    result.col0 = R.col0 * invInertiaLocal.x;
    result.col1 = R.col1 * invInertiaLocal.y;
    result.col2 = R.col2 * invInertiaLocal.z;
    return result; // NOTE: apply as R_scaled * (R^T * v), see applyImpulseAtPoint
}

void RigidBody::applyImpulseAtPoint(const Vec3& impulse, const Vec3& worldPoint) {
    if (isStatic) return;
    linearVelocity = linearVelocity + impulse * invMass;

    Vec3 r = worldPoint - position;
    Vec3 torqueImpulse = r.cross(impulse);

    // angularVelocity += I_world^-1 * torqueImpulse
    Mat3 R = orientation.toMat3();
    Vec3 local = R.transformInverse(torqueImpulse); // into local space
    Vec3 localScaled = {
        local.x * invInertiaLocal.x,
        local.y * invInertiaLocal.y,
        local.z * invInertiaLocal.z
    };
    Vec3 worldDeltaAngVel = R * localScaled; // back into world space
    angularVelocity = angularVelocity + worldDeltaAngVel;
}

void RigidBody::integrate(float dt) {
    if (isStatic) return;

    Vec3 acceleration = forceAccum * invMass;
    linearVelocity = linearVelocity + acceleration * dt;
    position = position + linearVelocity * dt;

    // Angular acceleration via world-space inverse inertia (torque already
    // accumulated in world space; convert -> local -> scale -> back to world)
    Mat3 R = orientation.toMat3();
    Vec3 localTorque = R.transformInverse(torqueAccum);
    Vec3 localAngAccel = {
        localTorque.x * invInertiaLocal.x,
        localTorque.y * invInertiaLocal.y,
        localTorque.z * invInertiaLocal.z
    };
    Vec3 worldAngAccel = R * localAngAccel;

    angularVelocity = angularVelocity + worldAngAccel * dt;
    orientation = Quat::integrate(orientation, angularVelocity, dt);

    forceAccum = {};
    torqueAccum = {};
}
