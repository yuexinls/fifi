#pragma once
#include "ContactManifold.h"
#include "physics/RigidBody.h"
#include <vector>
#include <algorithm>
#include <cmath>

// clip polygon against plane
static std::vector<Vec3> clipPolygonByPlane(
    const std::vector<Vec3>& poly,
    const Vec3& n, float d)
{
    std::vector<Vec3> out;
    if (poly.empty()) return out;

    int count = (int)poly.size();
    for (int i = 0; i < count; i++) {
        const Vec3& curr = poly[i];
        const Vec3& next = poly[(i + 1) % count];

        float dCurr = n.dot(curr) - d;
        float dNext = n.dot(next) - d;

        if (dCurr <= 0.0f) out.push_back(curr);
        if ((dCurr < 0.0f) != (dNext < 0.0f)) {
            float t = dCurr / (dCurr - dNext);
            out.push_back(curr + (next - curr) * t);
        }
    }
    return out;
}

inline void generateBoxContacts(ContactManifold& m,
                                 const RigidBody& A,
                                 const RigidBody& B)
{
    // n points from B toward A
    const Vec3& n = m.normal;

    Vec3 axesA[3] = {
        A.orientation.rotate({1,0,0}),
        A.orientation.rotate({0,1,0}),
        A.orientation.rotate({0,0,1})
    };
    float halfA[3] = {
        A.collider.halfExtents.x,
        A.collider.halfExtents.y,
        A.collider.halfExtents.z
    };

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

    int   refAxis = 0;
    float bestDot = -1.0f;
    for (int i = 0; i < 3; i++) {
        float d = std::abs(axesA[i].dot(n));
        if (d > bestDot) { bestDot = d; refAxis = i; }
    }

    // if axis is aligned with n, flip it
    Vec3 refNormal = (axesA[refAxis].dot(n) > 0.0f)
                   ? -axesA[refAxis]
                   :  axesA[refAxis];
    Vec3 refCenter = A.position + refNormal * halfA[refAxis];

    // tangent axes of reference face
    int   ti1 = (refAxis + 1) % 3;
    int   ti2 = (refAxis + 2) % 3;
    Vec3  t1  = axesA[ti1];
    Vec3  t2  = axesA[ti2];
    float h1  = halfA[ti1];
    float h2  = halfA[ti2];

    int   incAxis  = 0;
    float bestIncDot = -1.0f;
    for (int i = 0; i < 3; i++) {
        float d = std::abs(axesB[i].dot(n));
        if (d > bestIncDot) { bestIncDot = d; incAxis = i; }
    }

    // if axis is anti-aligned with n, flip it
    Vec3 incNormal = (axesB[incAxis].dot(n) < 0.0f)
                   ? -axesB[incAxis]
                   :  axesB[incAxis];
    Vec3 incCenter = B.position + incNormal * halfB[incAxis];

    int   ii1 = (incAxis + 1) % 3;
    int   ii2 = (incAxis + 2) % 3;
    Vec3  it1 = axesB[ii1];
    Vec3  it2 = axesB[ii2];
    float ih1 = halfB[ii1];
    float ih2 = halfB[ii2];

    // build incident face polygon
    std::vector<Vec3> poly = {
        incCenter + it1*ih1 + it2*ih2,
        incCenter - it1*ih1 + it2*ih2,
        incCenter - it1*ih1 - it2*ih2,
        incCenter + it1*ih1 - it2*ih2
    };

    // clip against 4 side planes of the reference face
    // dot(n, p) <= dot(n, refCenter) + half
    poly = clipPolygonByPlane(poly,  t1,  t1.dot(refCenter) + h1);
    if (poly.empty()) return;
    poly = clipPolygonByPlane(poly, -t1, -t1.dot(refCenter) + h1);
    if (poly.empty()) return;
    poly = clipPolygonByPlane(poly,  t2,  t2.dot(refCenter) + h2);
    if (poly.empty()) return;
    poly = clipPolygonByPlane(poly, -t2, -t2.dot(refCenter) + h2);
    if (poly.empty()) return;

    // depth test:
    // refNormal points from A toward B
    // depth = refNormal.dot(refCenter) - refNormal.dot(p)
    // depth > 0 means p is on the B side of A's contact face (penetrating)
    float refD = refNormal.dot(refCenter);

    m.contacts.clear();
    for (auto& p : poly) {
        float depth = refD - refNormal.dot(p);
        if (depth > -0.001f) {
            ContactPoint cp;
            cp.position         = p + refNormal * std::max(depth, 0.0f);
            cp.penetrationDepth = std::max(depth, 0.0f);
            m.contacts.push_back(cp);
        }
    }

    // keep at most 4 deepest contact points
    if (m.contacts.size() > 4) {
        std::sort(m.contacts.begin(), m.contacts.end(),
            [](const ContactPoint& a, const ContactPoint& b) {
                return a.penetrationDepth > b.penetrationDepth;
            });
        m.contacts.resize(4);
    }

    if (m.contacts.empty()) return;

    // update manifold summary
    Vec3  avgPos   = {};
    float maxDepth = 0.0f;
    for (auto& cp : m.contacts) {
        avgPos   += cp.position;
        maxDepth  = std::max(maxDepth, cp.penetrationDepth);
    }
    m.contactPoint     = avgPos * (1.0f / (float)m.contacts.size());
    m.penetrationDepth = maxDepth;
}

inline void enrichManifold(ContactManifold& m,
                            const RigidBody& A,
                            const RigidBody& B)
{
    // always seed with EPA result as fallback
    ContactPoint epaCp;
    epaCp.position         = m.contactPoint;
    epaCp.penetrationDepth = m.penetrationDepth;
    m.contacts             = { epaCp };

    if (A.collider.type == Collider::Type::Box &&
        B.collider.type == Collider::Type::Box)
    {
        std::vector<ContactPoint> backup = m.contacts;
        float                     backupDepth = m.penetrationDepth;

        generateBoxContacts(m, A, B);

        // fall back to EPA point if clipping failed or produced garbage
        if (m.contacts.empty() || m.penetrationDepth > 1.0f) {
            m.contacts         = backup;
            m.penetrationDepth = backupDepth;
            m.contactPoint     = epaCp.position;
        }
    }
    // spheres keep the single EPA point
}


