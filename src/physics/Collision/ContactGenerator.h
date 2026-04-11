#pragma once
#include "ContactManifold.h"
#include "physics/RigidBody.h"
#include <vector>
#include <algorithm>
#include <cmath>

// clip a polygon by a plane, returns the portion in front of the plane (Sutherland-Hodgman step)
static std::vector<Vec3> clipByPlane(const std::vector<Vec3>& poly,
                                     const Vec3& planeNormal,
                                     float planeDist)
{
    std::vector<Vec3> out;
    if (poly.empty()) return out;

    for (int i = 0; i < (int)poly.size(); i++) {
        const Vec3& curr = poly[i];
        const Vec3& next = poly[(i + 1) % poly.size()];

        float dCurr = planeNormal.dot(curr) - planeDist;
        float dNext = planeNormal.dot(next) - planeDist;

        if (dCurr >= 0.0f) out.push_back(curr);

        // edge intersects plane
        if ((dCurr >= 0.0f) != (dNext >= 0.0f)) {
            float t = dCurr / (dCurr - dNext);
            out.push_back(curr + (next - curr) * t);
        }
    }
    return out;
}

// generate a contact manifold for box-box pairs using clipping
inline void generateBoxContacts(ContactManifold& manifold,
                                 const RigidBody& A,
                                 const RigidBody& B)
{
    Vec3 n = manifold.normal; // points from B toward A

    // find reference face on A (most aligned with n) and incident face on B (most anti-aligned with n)
    Vec3 axes[3] = {
        A.orientation.rotate({1,0,0}),
        A.orientation.rotate({0,1,0}),
        A.orientation.rotate({0,0,1})
    };
    float halfA[3] = {
        A.collider.halfExtents.x,
        A.collider.halfExtents.y,
        A.collider.halfExtents.z
    };

    int refAxis   = 0;
    float bestDot = -1.0f;
    for (int i = 0; i < 3; i++) {
        float d = std::abs(axes[i].dot(n));
        if (d > bestDot) { bestDot = d; refAxis = i; }
    }

    Vec3 refNormal = axes[refAxis].dot(n) > 0 ? axes[refAxis] : -axes[refAxis];
    Vec3 refCenter = A.position + refNormal * halfA[refAxis];

    // reference face basis
    Vec3 t1 = axes[(refAxis + 1) % 3];
    Vec3 t2 = axes[(refAxis + 2) % 3];
    float h1 = halfA[(refAxis + 1) % 3];
    float h2 = halfA[(refAxis + 2) % 3];

    // find incident face on B (most anti-aligned with n)
    Vec3 axesB[3] = {
        B.orientation.rotate({1,0,0}),
        B.orientation.rotate({0,1,0}),
        B.orientation.rotate({0,0,1})
    };
    float halfB[3] = {
        B.collider.halfExtents.x,
        B.collider.halfExtents.y,
        B.collider.halfExtents.z
    };

    int incAxis   = 0;
    float worstDot = 1.0f;
    for (int i = 0; i < 3; i++) {
        float d = axesB[i].dot(n);
        if (d < worstDot) { worstDot = d; incAxis = i; }
    }

    Vec3 incNormal = axesB[incAxis].dot(n) < 0 ? axesB[incAxis] : -axesB[incAxis];
    Vec3 incCenter = B.position + incNormal * halfB[incAxis];

    Vec3 it1 = axesB[(incAxis + 1) % 3];
    Vec3 it2 = axesB[(incAxis + 2) % 3];
    float ih1 = halfB[(incAxis + 1) % 3];
    float ih2 = halfB[(incAxis + 2) % 3];

    // incident face polygon (4 corners)
    std::vector<Vec3> poly = {
        incCenter + it1*ih1 + it2*ih2,
        incCenter - it1*ih1 + it2*ih2,
        incCenter - it1*ih1 - it2*ih2,
        incCenter + it1*ih1 - it2*ih2
    };

    // clip against the 4 side planes of the reference face (defined by t1,t2,h1,h2)
    struct Plane { Vec3 n; float d; };
    Plane sidePlanes[4] = {
        {  t1,  t1.dot(refCenter) + h1 },
        { -t1, -t1.dot(refCenter) + h1 },
        {  t2,  t2.dot(refCenter) + h2 },
        { -t2, -t2.dot(refCenter) + h2 }
    };

    for (auto& plane : sidePlanes) {
        poly = clipByPlane(poly, plane.n, plane.d);
        if (poly.empty()) return;
    }

    // keep only points behind the reference face, and build contact points
    float refD = refNormal.dot(refCenter);
    manifold.contacts.clear();

    for (auto& p : poly) {
        float depth = refD - refNormal.dot(p);
        if (depth >= -0.01f) {
            Vec3 projected = p + refNormal * std::max(depth, 0.0f);
            ContactPoint cp;
            cp.position         = projected;
            cp.penetrationDepth = std::max(depth, 0.0f);
            manifold.contacts.push_back(cp);
        }
    }

    // update manifold contact point and penetration depth as average of contacts for resolution
    if (!manifold.contacts.empty()) {
        Vec3  avgPos   = {};
        float maxDepth = 0.0f;
        for (auto& cp : manifold.contacts) {
            avgPos   += cp.position;
            maxDepth  = std::max(maxDepth, cp.penetrationDepth);
        }
        manifold.contactPoint     = avgPos * (1.0f / manifold.contacts.size());
        manifold.penetrationDepth = maxDepth;
    }
}

// called after GJK & EPA to add multiple contact points for box-box pairs
inline void enrichManifold(ContactManifold& m,
                            const RigidBody& A,
                            const RigidBody& B)
{
    bool aBox = (A.collider.type == Collider::Type::Box);
    bool bBox = (B.collider.type == Collider::Type::Box);

    if (aBox && bBox) {
        generateBoxContacts(m, A, B);
        return;
    }

    // for sphere contacts, just use the single contact point from EPA
    ContactPoint cp;
    cp.position         = m.contactPoint;
    cp.penetrationDepth = m.penetrationDepth;
    m.contacts          = { cp };
}


