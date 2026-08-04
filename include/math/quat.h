#pragma once
#include "vec3.h"

struct Mat3 {
    // Column-major 3x3, columns are the local x/y/z axes in world space
    Vec3 col0{1,0,0}, col1{0,1,0}, col2{0,0,1};

    Vec3 operator*(const Vec3& v) const {
        return col0 * v.x + col1 * v.y + col2 * v.z;
    }
    // Transform a world-space vector into this frame's local space (transpose, since orthonormal)
    Vec3 transformInverse(const Vec3& v) const {
        return { col0.dot(v), col1.dot(v), col2.dot(v) };
    }
};

struct Quat {
    float x = 0, y = 0, z = 0, w = 1;

    static Quat identity() { return {0, 0, 0, 1}; }

    Quat operator*(const Quat& o) const {
        return {
            w*o.x + x*o.w + y*o.z - z*o.y,
            w*o.y - x*o.z + y*o.w + z*o.x,
            w*o.z + x*o.y - y*o.x + z*o.w,
            w*o.w - x*o.x - y*o.y - z*o.z
        };
    }

    Quat normalized() const {
        float l = std::sqrt(x*x + y*y + z*z + w*w);
        if (l < 1e-8f) return identity();
        return {x/l, y/l, z/l, w/l};
    }

    static Quat integrate(const Quat& q, const Vec3& angVel, float dt) {
        Quat dq = { angVel.x*dt*0.5f, angVel.y*dt*0.5f, angVel.z*dt*0.5f, 0.0f };
        Quat r = dq * q;
        return Quat{ q.x + r.x, q.y + r.y, q.z + r.z, q.w + r.w }.normalized();
    }

    // Build the rotation matrix represented by this quaternion
    Mat3 toMat3() const {
        Mat3 m;
        float xx = x*x, yy = y*y, zz = z*z;
        float xy = x*y, xz = x*z, yz = y*z;
        float wx = w*x, wy = w*y, wz = w*z;
        m.col0 = { 1 - 2*(yy+zz), 2*(xy+wz),     2*(xz-wy) };
        m.col1 = { 2*(xy-wz),     1 - 2*(xx+zz), 2*(yz+wx) };
        m.col2 = { 2*(xz+wy),     2*(yz-wx),     1 - 2*(xx+yy) };
        return m;
    }
};
