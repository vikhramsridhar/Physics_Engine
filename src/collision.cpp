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
// Box vs Box — Separating Axis Theorem, FACE AXES ONLY.
//
// Educational simplification: a fully correct SAT implementation also tests
// the 9 cross-product axes of edge pairs (needed to catch certain edge-edge
// collision configurations). Skipping those means some edge-on-edge contacts
// will be missed or slightly mispositioned. This is a reasonable place to
// start; the fix is a good follow-up exercise (see README).
// ---------------------------------------------------------------------------
static bool testAxis(const Vec3& axis, RigidBody& a, RigidBody& b,
                      float& bestOverlap, Vec3& bestAxis) {
    float len = axis.length();
    if (len < 1e-6f) return true; // degenerate axis, skip
    Vec3 n = axis * (1.0f / len);

    Mat3 rotA = a.orientation.toMat3();
    Mat3 rotB = b.orientation.toMat3();

    // Project each box's half-extents onto axis n to get its "radius" along n
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

    if (overlap < 0.0f) return false; // separating axis found -> no collision

    if (overlap < bestOverlap) {
        bestOverlap = overlap;
        // Keep the axis pointing from A to B
        bestAxis = centerDist < 0.0f ? -n : n;
    }
    return true;
}

bool boxBoxContact(RigidBody& a, RigidBody& b, Contact& out) {
    Mat3 rotA = a.orientation.toMat3();
    Mat3 rotB = b.orientation.toMat3();

    float bestOverlap = FLT_MAX;
    Vec3 bestAxis;

    // 3 face axes of A, 3 face axes of B
    Vec3 axes[6] = { rotA.col0, rotA.col1, rotA.col2, rotB.col0, rotB.col1, rotB.col2 };
    for (auto& axis : axes) {
        if (!testAxis(axis, a, b, bestOverlap, bestAxis)) return false;
    }

    out.a = &a; out.b = &b;
    out.normal = bestAxis;
    out.penetration = bestOverlap;
    // Approximate contact point: midpoint between the two centers, projected
    // onto the separating plane. Good enough for stacking demos; a precise
    // implementation would clip the incident face against the reference face.
    out.point = a.position + (b.position - a.position) * 0.5f;
    return true;
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
            Contact c;
            bool hit = false;

            if (a.shape.type == ShapeType::Sphere && b.shape.type == ShapeType::Sphere) {
                hit = sphereSphereContact(a, b, c);
            } else if (a.shape.type == ShapeType::Box && b.shape.type == ShapeType::Box) {
                hit = boxBoxContact(a, b, c);
            } else if (a.shape.type == ShapeType::Sphere && b.shape.type == ShapeType::Box) {
                hit = sphereBoxContact(a, b, c); // (sphereBody=a, boxBody=b)
            } else { // a = Box, b = Sphere
                hit = sphereBoxContact(b, a, c); // (sphereBody=b, boxBody=a)
            }
            if (hit) contactsOut.push_back(c);
        }
    }
}
