#pragma once
#include "ContactManifold.h"
#include "physics/RigidBody.h"
#include <vector>
#include <algorithm>
#include <cmath>

// clip polygon by plane defined by normal n and distance d from origin
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

        if (dCurr <= 0.0f) out.push_back(curr); // inside
        if ((dCurr < 0.0f) != (dNext < 0.0f)) { // edge crosses
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
    const Vec3& n = m.normal; // points from B toward A

    // find the reference face on A (the one most aligned with the contact normal)
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

    int   refAxis = 0;
    float bestDot = -1.0f;
    for (int i = 0; i < 3; i++) {
        float d = std::abs(axesA[i].dot(n));
        if (d > bestDot) { bestDot = d; refAxis = i; }
    }

    // reference face normal should point toward B
    bool  refFlip   = axesA[refAxis].dot(n) < 0.0f;
    Vec3  refNormal = refFlip ? -axesA[refAxis] : axesA[refAxis];
    Vec3  refCenter = A.position + refNormal * halfA[refAxis];

    // tangent axes for clipping planes
    int   ti1  = (refAxis + 1) % 3;
    int   ti2  = (refAxis + 2) % 3;
    Vec3  t1   = axesA[ti1];
    Vec3  t2   = axesA[ti2];
    float h1   = halfA[ti1];
    float h2   = halfA[ti2];

    // find the incident face on B (the one most anti-aligned with the contact normal)
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

    int   incAxis  = 0;
    float mostNeg  = 1.0f;
    for (int i = 0; i < 3; i++) {
        float d = axesB[i].dot(n);
        if (d < mostNeg) { mostNeg = d; incAxis = i; }
    }

    bool  incFlip   = axesB[incAxis].dot(n) > 0.0f;
    Vec3  incNormal = incFlip ? -axesB[incAxis] : axesB[incAxis];
    Vec3  incCenter = B.position + incNormal * halfB[incAxis];

    int   ii1  = (incAxis + 1) % 3;
    int   ii2  = (incAxis + 2) % 3;
    Vec3  it1  = axesB[ii1];
    Vec3  it2  = axesB[ii2];
    float ih1  = halfB[ii1];
    float ih2  = halfB[ii2];

    // build incident face as a box centered on incCenter, with axes it1/it2 and extents ih1/ih2
    std::vector<Vec3> poly = {
        incCenter + it1*ih1 + it2*ih2,
        incCenter - it1*ih1 + it2*ih2,
        incCenter - it1*ih1 - it2*ih2,
        incCenter + it1*ih1 - it2*ih2
    };

    // clip incident face against the 4 planes of the reference face (t1/t2 with h1/h2)
    poly = clipPolygonByPlane(poly,  t1,  t1.dot(refCenter) + h1);
    if (poly.empty()) return;
    poly = clipPolygonByPlane(poly, -t1, -t1.dot(refCenter) + h1);
    if (poly.empty()) return;
    poly = clipPolygonByPlane(poly,  t2,  t2.dot(refCenter) + h2);
    if (poly.empty()) return;
    poly = clipPolygonByPlane(poly, -t2, -t2.dot(refCenter) + h2);
    if (poly.empty()) return;

    // keep only points that are behind the reference face (penetrating)
    float refD = refNormal.dot(refCenter);

    m.contacts.clear();

    for (auto& p : poly) {
        float depth = refD - refNormal.dot(p);
        if (depth > -0.005f) { // small tolerance
            ContactPoint cp;
            cp.position         = p + refNormal * std::max(depth, 0.0f);
            cp.penetrationDepth = std::max(depth, 0.0f);
            m.contacts.push_back(cp);
        }
    }

    // reduce to max 4 contact points by depth
    if (m.contacts.size() > 4) {
        std::sort(m.contacts.begin(), m.contacts.end(),
            [](const ContactPoint& a, const ContactPoint& b){
                return a.penetrationDepth > b.penetrationDepth;
            });
        m.contacts.resize(4);
    }

    // update manifold contact point and penetration depth as the average of the contact points
    if (m.contacts.empty()) return;

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
    // always start with EPA
    ContactPoint epaCp;
    epaCp.position         = m.contactPoint;
    epaCp.penetrationDepth = m.penetrationDepth;
    m.contacts             = { epaCp };

    bool aBox = (A.collider.type == Collider::Type::Box);
    bool bBox = (B.collider.type == Collider::Type::Box);

    if (aBox && bBox) {
        // try full clipping-based contact generation
        std::vector<ContactPoint> backup = m.contacts;
        generateBoxContacts(m, A, B);

        // safety check
        if (m.contacts.empty() || m.penetrationDepth > 2.0f) {
            m.contacts         = backup;
            m.contactPoint     = epaCp.position;
            m.penetrationDepth = epaCp.penetrationDepth;
        }
    }
}


