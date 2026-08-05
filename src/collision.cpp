#include "../include/physics/collision.h"
#include <cfloat>
#include <cmath>

// ---------------------------------------------------------------------------
// Sphere vs Sphere
// ---------------------------------------------------------------------------
bool sphereSphereContact(RigidBody& a, RigidBody& b, Contact& out) {
    Vec3 delta = b.position - a.position;
    float dist = delta.length();
    float radiusSum = a.shape.radius + b.shape.radius;
    if (dist >= radiusSum || dist < 1e-6f) return false;

    out.a = &a; out.b = &b;
    out.normal = delta * (1.0f / dist);
    out.penetration = radiusSum - dist;
    out.point = a.position + out.normal * a.shape.radius;
    return true;
}

// ---------------------------------------------------------------------------
// Sphere vs Box
// Find the closest point on the (oriented) box to the sphere center, in the
// box's local space, then test distance to that point against the radius.
// ---------------------------------------------------------------------------
bool sphereBoxContact(RigidBody& sphereBody, RigidBody& boxBody, Contact& out) {
    Mat3 boxRot = boxBody.orientation.toMat3();
    Vec3 localSphereCenter = boxRot.transformInverse(sphereBody.position - boxBody.position);

    Vec3 he = boxBody.shape.halfExtents;
    Vec3 closestLocal = {
        std::max(-he.x, std::min(he.x, localSphereCenter.x)),
        std::max(-he.y, std::min(he.y, localSphereCenter.y)),
        std::max(-he.z, std::min(he.z, localSphereCenter.z))
    };

    Vec3 localDelta = localSphereCenter - closestLocal;
    float distSq = localDelta.dot(localDelta);
    float r = sphereBody.shape.radius;
    if (distSq >= r * r) return false;

    float dist = std::sqrt(distSq);
    Vec3 localNormal = dist > 1e-6f ? localDelta * (1.0f / dist) : Vec3{0, 1, 0};

    out.a = &boxBody; out.b = &sphereBody;
    out.normal = boxRot * localNormal; // box -> sphere, world space
    out.penetration = r - dist;
    out.point = boxBody.position + boxRot * closestLocal;
    return true;
}
// ---------------------------------------------------------------------------
// Box vs Box — FULL Separating Axis Theorem (face axes + edge-edge axes)
// with proper contact manifold generation via face clipping.
//
// This replaces an earlier single-approximate-point version, which caused
// box stacks to gain energy and explode (see README history). The fix has
// two parts:
//   1. Test all 15 SAT axes (6 face + 9 edge cross-products), not just 6,
//      so penetration depth/direction is correct in edge-on-edge cases.
//   2. When the minimum-penetration axis is a FACE axis (the common case
//      for stacked/resting boxes), generate a proper 2-4 point contact
//      manifold by clipping the incident face against the reference face's
//      four side planes (Sutherland-Hodgman clipping) rather than using
//      one point at the midpoint of the two centers. Multiple contact
//      points let the solver resist rotation correctly, which is what
//      was missing before and caused the instability.
//   When the axis is an EDGE axis, we fall back to a single contact point
//   near the closest approach of the two edges -- edge-edge contact is
//   inherently a single point in exact geometry anyway.
// ---------------------------------------------------------------------------

namespace {

struct AxisResult {
    float overlap = FLT_MAX;
    Vec3 axis;               // pointing from A to B
    int type = -1;            // 0 = face of A, 1 = face of B, 2 = edge-edge
    int faceAxisIndex = -1;
    int edgeIndexA = -1, edgeIndexB = -1;
};

bool testAxisFull(const Vec3& axis, RigidBody& a, RigidBody& b,
                   int type, int faceIdx, int edgeA, int edgeB,
                   AxisResult& best) {
    float len = axis.length();
    if (len < 1e-6f) return true; // degenerate (near-parallel edges), skip
    Vec3 n = axis * (1.0f / len);

    Mat3 rotA = a.orientation.toMat3();
    Mat3 rotB = b.orientation.toMat3();

    auto boxRadius = [&](RigidBody& box, Mat3& rot) {
        Vec3 he = box.shape.halfExtents;
        return std::abs(rot.col0.dot(n)) * he.x
             + std::abs(rot.col1.dot(n)) * he.y
             + std::abs(rot.col2.dot(n)) * he.z;
    };

    float ra = boxRadius(a, rotA);
    float rb = boxRadius(b, rotB);
    float centerDist = (b.position - a.position).dot(n);
    float overlap = ra + rb - std::abs(centerDist);

    if (overlap < 0.0f) return false; // separating axis -> no collision

    if (overlap < best.overlap) {
        best.overlap = overlap;
        best.axis = centerDist < 0.0f ? -n : n;
        best.type = type;
        best.faceAxisIndex = faceIdx;
        best.edgeIndexA = edgeA;
        best.edgeIndexB = edgeB;
    }
    return true;
}

// Returns the 4 world-space vertices of the face of `box` whose outward
// normal is closest to `dir`.
void getFaceVerts(RigidBody& box, const Vec3& dir, Vec3 outVerts[4]) {
    Mat3 rot = box.orientation.toMat3();
    Vec3 he = box.shape.halfExtents;
    Vec3 axes[3] = { rot.col0, rot.col1, rot.col2 };
    float extents[3] = { he.x, he.y, he.z };

    int bestAxis = 0; float bestSign = 1.0f; float bestDot = -FLT_MAX;
    for (int i = 0; i < 3; ++i) {
        float d = axes[i].dot(dir);
        if (std::abs(d) > bestDot) { bestDot = std::abs(d); bestAxis = i; bestSign = d > 0 ? 1.0f : -1.0f; }
    }

    int u = (bestAxis + 1) % 3, v = (bestAxis + 2) % 3;
    Vec3 center = box.position + axes[bestAxis] * (extents[bestAxis] * bestSign);
    Vec3 du = axes[u] * extents[u];
    Vec3 dv = axes[v] * extents[v];
    outVerts[0] = center - du - dv;
    outVerts[1] = center + du - dv;
    outVerts[2] = center + du + dv;
    outVerts[3] = center - du + dv;
}

// Sutherland-Hodgman: clips polygon `poly` (n verts) against a single plane
// (point + inward-pointing normal), keeping the inside portion.
int clipPolygonAgainstPlane(const Vec3* poly, int n, const Vec3& planePoint,
                             const Vec3& planeNormal, Vec3* out) {
    int count = 0;
    for (int i = 0; i < n; ++i) {
        Vec3 curr = poly[i];
        Vec3 next = poly[(i + 1) % n];
        float dCurr = (curr - planePoint).dot(planeNormal);
        float dNext = (next - planePoint).dot(planeNormal);

        if (dCurr >= 0.0f) out[count++] = curr;
        if ((dCurr >= 0.0f) != (dNext >= 0.0f)) {
            float t = dCurr / (dCurr - dNext);
            out[count++] = curr + (next - curr) * t;
        }
    }
    return count;
}

} // namespace

bool boxBoxContact(RigidBody& a, RigidBody& b, std::vector<Contact>& out) {
    Mat3 rotA = a.orientation.toMat3();
    Mat3 rotB = b.orientation.toMat3();
    Vec3 axesA[3] = { rotA.col0, rotA.col1, rotA.col2 };
    Vec3 axesB[3] = { rotB.col0, rotB.col1, rotB.col2 };

    AxisResult best;

    for (int i = 0; i < 3; ++i)
        if (!testAxisFull(axesA[i], a, b, 0, i, -1, -1, best)) return false;
    for (int i = 0; i < 3; ++i)
        if (!testAxisFull(axesB[i], a, b, 1, i, -1, -1, best)) return false;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            if (!testAxisFull(axesA[i].cross(axesB[j]), a, b, 2, -1, i, j, best)) return false;

    if (best.type == -1) return false; // shouldn't happen; safety net

    // --- Case 1: face contact -> clip incident face against reference face ---
    if (best.type == 0 || best.type == 1) {
        RigidBody& refBox = (best.type == 0) ? a : b;
        RigidBody& incBox = (best.type == 0) ? b : a;
        Vec3 refNormal = (best.type == 0) ? best.axis : best.axis * -1.0f;

        Vec3 refFace[4], incFace[4];
        getFaceVerts(refBox, refNormal, refFace);
        getFaceVerts(incBox, refNormal * -1.0f, incFace);

        Vec3 refCenter = (refFace[0] + refFace[1] + refFace[2] + refFace[3]) * 0.25f;
        Vec3 poly[16]; int polyCount = 4;
        for (int i = 0; i < 4; ++i) poly[i] = incFace[i];

        for (int e = 0; e < 4; ++e) {
            Vec3 edgeStart = refFace[e];
            Vec3 edgeEnd = refFace[(e + 1) % 4];
            Vec3 edgeDir = (edgeEnd - edgeStart).normalized();
            Vec3 sideNormal = edgeDir.cross(refNormal);
            if (sideNormal.dot(refCenter - edgeStart) < 0.0f) sideNormal = -sideNormal;

            Vec3 clipped[16];
            int clippedCount = clipPolygonAgainstPlane(poly, polyCount, edgeStart, sideNormal, clipped);
            polyCount = std::min(clippedCount, 16);
            for (int i = 0; i < polyCount; ++i) poly[i] = clipped[i];
            if (polyCount == 0) break;
        }

        bool anyPoint = false;
        for (int i = 0; i < polyCount; ++i) {
            float dist = (poly[i] - refFace[0]).dot(refNormal);
            if (dist >= 0.0f) continue; // not actually penetrating
            Contact c;
            c.a = &a; c.b = &b;
            c.normal = best.axis;
            c.penetration = -dist;
            c.point = poly[i];
            out.push_back(c);
            anyPoint = true;
        }

        if (!anyPoint) { // rare degenerate/grazing case -- still resolve something
            Contact c;
            c.a = &a; c.b = &b;
            c.normal = best.axis;
            c.penetration = best.overlap;
            c.point = a.position + (b.position - a.position) * 0.5f;
            out.push_back(c);
        }
        return true;
    }

    // --- Case 2: edge-edge contact -> single point near closest approach ---
    {
        Vec3 he = a.shape.halfExtents;
        Vec3 pointOnA = a.position;
        for (int k = 0; k < 3; ++k) {
            if (k == best.edgeIndexA) continue;
            float sign = (b.position - a.position).dot(axesA[k]) > 0 ? 1.0f : -1.0f;
            pointOnA = pointOnA + axesA[k] * (he.component(k) * sign);
        }
        Vec3 heB = b.shape.halfExtents;
        Vec3 pointOnB = b.position;
        for (int k = 0; k < 3; ++k) {
            if (k == best.edgeIndexB) continue;
            float sign = (a.position - b.position).dot(axesB[k]) > 0 ? 1.0f : -1.0f;
            pointOnB = pointOnB + axesB[k] * (heB.component(k) * sign);
        }

        Contact c;
        c.a = &a; c.b = &b;
        c.normal = best.axis;
        c.penetration = best.overlap;
        c.point = (pointOnA + pointOnB) * 0.5f;
        out.push_back(c);
        return true;
    }
}


// ---------------------------------------------------------------------------
// Sphere vs implicit ground plane (y = groundY)
// ---------------------------------------------------------------------------
bool spherePlaneContact(RigidBody& sphereBody, RigidBody& groundBody, float groundY, Contact& out) {
    float dist = sphereBody.position.y - groundY;
    if (dist >= sphereBody.shape.radius) return false;

    out.a = &groundBody; out.b = &sphereBody;
    out.normal = {0, 1, 0};
    out.penetration = sphereBody.shape.radius - dist;
    out.point = { sphereBody.position.x, groundY, sphereBody.position.z };
    return true;
}

// ---------------------------------------------------------------------------
// Box vs implicit ground plane.
//
// IMPORTANT: this deliberately emits AT MOST ONE contact per box, not one
// per penetrating corner. An earlier version emitted up to 4 (one per
// penetrating corner); the solver then treated those as four independent,
// fully redundant constraints on the same vertical DOF. Sequential impulse
// with several redundant constraints and no relaxation/averaging diverges
// instead of converging -- each pass over the 4 corners re-corrects the
// same motion the others just corrected, compounding rather than settling.
// This is a well-known failure mode of naive multi-point contact handling.
//
// The fix here: find every penetrating corner, then reduce them to a
// single contact using the deepest penetration and the AVERAGE contact
// point. This loses some rotational accuracy for a tilted box hitting one
// corner (a real engine keeps a small manifold, typically up to 4 points,
// but re-uses persistent point IDs across frames and relaxes redundant
// rows -- meaningfully more machinery than fits here). For a box resting
// flat or falling face-first, which is the common case, this is stable.
// ---------------------------------------------------------------------------
bool boxPlaneContact(RigidBody& boxBody, RigidBody& groundBody, float groundY, std::vector<Contact>& out) {
    Mat3 rot = boxBody.orientation.toMat3();
    Vec3 he = boxBody.shape.halfExtents;

    float maxPenetration = 0.0f;
    Vec3 avgPoint{};
    int count = 0;

    for (int sx = -1; sx <= 1; sx += 2)
    for (int sy = -1; sy <= 1; sy += 2)
    for (int sz = -1; sz <= 1; sz += 2) {
        Vec3 localCorner = { he.x * sx, he.y * sy, he.z * sz };
        Vec3 worldCorner = boxBody.position + rot * localCorner;

        float dist = worldCorner.y - groundY;
        if (dist >= 0.0f) continue;

        maxPenetration = std::max(maxPenetration, -dist);
        avgPoint = avgPoint + worldCorner;
        ++count;
    }

    if (count == 0) return false;
    avgPoint = avgPoint * (1.0f / (float)count);
    // Keep the point's height at the ground so the contact sits on the
    // surface rather than partway inside the box.
    avgPoint.y = groundY;

    Contact c;
    c.a = &groundBody; c.b = &boxBody;
    c.normal = {0, 1, 0};
    c.penetration = maxPenetration;
    c.point = avgPoint;
    out.push_back(c);
    return true;
}

// ---------------------------------------------------------------------------
// Dispatch
// ---------------------------------------------------------------------------
void generateContacts(std::vector<RigidBody>& bodies, RigidBody& groundBody,
                       float groundY, std::vector<Contact>& contactsOut) {
    for (auto& body : bodies) {
        Contact c;
        if (body.shape.type == ShapeType::Sphere) {
            if (spherePlaneContact(body, groundBody, groundY, c)) contactsOut.push_back(c);
        } else {
            std::vector<Contact> boxContacts;
            if (boxPlaneContact(body, groundBody, groundY, boxContacts))
                for (auto& bc : boxContacts) contactsOut.push_back(bc);
        }
    }

    for (size_t i = 0; i < bodies.size(); ++i) {
        for (size_t j = i + 1; j < bodies.size(); ++j) {
            RigidBody& a = bodies[i];
            RigidBody& b = bodies[j];

            if (a.shape.type == ShapeType::Sphere && b.shape.type == ShapeType::Sphere) {
                Contact c;
                if (sphereSphereContact(a, b, c)) contactsOut.push_back(c);
            } else if (a.shape.type == ShapeType::Box && b.shape.type == ShapeType::Box) {
                boxBoxContact(a, b, contactsOut); // appends 1-4 points directly
            } else if (a.shape.type == ShapeType::Sphere && b.shape.type == ShapeType::Box) {
                Contact c;
                if (sphereBoxContact(a, b, c)) contactsOut.push_back(c); // (sphereBody=a, boxBody=b)
            } else { // a = Box, b = Sphere
                Contact c;
                if (sphereBoxContact(b, a, c)) contactsOut.push_back(c); // (sphereBody=b, boxBody=a)
            }
        }
    }
}
