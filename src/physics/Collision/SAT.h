#pragma once
#include "ContactManifold.h"
#include "physics/RigidBody.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

// projects a box onto an axis and returns (min, max)
static void projectBox(const Vec3& center,
                       const Vec3 axes[3],
                       const float halves[3],
                       const Vec3& axis,
                       float& outMin, float& outMax)
{
    float c = axis.dot(center);
    float r = std::abs(axis.dot(axes[0])) * halves[0]
            + std::abs(axis.dot(axes[1])) * halves[1]
            + std::abs(axis.dot(axes[2])) * halves[2];
    outMin = c - r;
    outMax = c + r;
}

// returns overlap on an axis, or negative if separated
static float axisOverlap(const Vec3& centerA, const Vec3 axesA[3], const float halfA[3],
                          const Vec3& centerB, const Vec3 axesB[3], const float halfB[3],
                          const Vec3& axis)
{
    float lenSq = axis.lengthSq();
    if (lenSq < 1e-10f) return std::numeric_limits<float>::max(); // skip degenerate

    Vec3  normAxis = axis * (1.0f / std::sqrt(lenSq));
    float minA, maxA, minB, maxB;
    projectBox(centerA, axesA, halfA, normAxis, minA, maxA);
    projectBox(centerB, axesB, halfB, normAxis, minB, maxB);

    float overlap = std::min(maxA, maxB) - std::max(minA, minB);
    return overlap; // negative = gap = no collision
}

// clip polygon by half-space (keep points where dot(n,p) <= d)
static std::vector<Vec3> satClip(const std::vector<Vec3>& poly,
                                  const Vec3& n, float d)
{
    std::vector<Vec3> out;
    if (poly.empty()) return out;
    int sz = (int)poly.size();
    for (int i = 0; i < sz; i++) {
        const Vec3& cur  = poly[i];
        const Vec3& next = poly[(i + 1) % sz];
        float dc = n.dot(cur)  - d;
        float dn = n.dot(next) - d;
        if (dc <= 0.0f) out.push_back(cur);
        if ((dc < 0.0f) != (dn < 0.0f)) {
            float t = dc / (dc - dn);
            out.push_back(cur + (next - cur) * t);
        }
    }
    return out;
}

// generate up to 4 contact points by clipping incident face
// against reference face side planes
static void generateSATContacts(ContactManifold& m,
                                  const RigidBody& A,
                                  const RigidBody& B,
                                  const Vec3& normal, // points A->B
                                  int refBodyAB)      // 0=A is ref, 1=B is ref
{
    // pick reference and incident body
    const RigidBody& Ref = (refBodyAB == 0) ? A : B;
    const RigidBody& Inc = (refBodyAB == 0) ? B : A;
    Vec3 n = (refBodyAB == 0) ? normal : -normal; // n points Ref->Inc

    Vec3  refAxes[3] = { Ref.orientation.rotate({1,0,0}),
                         Ref.orientation.rotate({0,1,0}),
                         Ref.orientation.rotate({0,0,1}) };
    float refHalf[3] = { Ref.collider.halfExtents.x,
                         Ref.collider.halfExtents.y,
                         Ref.collider.halfExtents.z };

    Vec3  incAxes[3] = { Inc.orientation.rotate({1,0,0}),
                         Inc.orientation.rotate({0,1,0}),
                         Inc.orientation.rotate({0,0,1}) };
    float incHalf[3] = { Inc.collider.halfExtents.x,
                         Inc.collider.halfExtents.y,
                         Inc.collider.halfExtents.z };

    // reference face (most aligned with n on Ref)
    int   refFace = 0;
    float bestDot = -1.0f;
    for (int i = 0; i < 3; i++) {
        float d = refAxes[i].dot(n);
        if (std::abs(d) > bestDot) { bestDot = std::abs(d); refFace = i; }
    }
    Vec3  refFaceNormal = (refAxes[refFace].dot(n) > 0.0f)
                        ?  refAxes[refFace]
                        : -refAxes[refFace];
    Vec3  refFaceCenter = Ref.position + refFaceNormal * refHalf[refFace];
    int   rt1 = (refFace + 1) % 3,  rt2 = (refFace + 2) % 3;
    Vec3  rT1 = refAxes[rt1],       rT2 = refAxes[rt2];
    float rH1 = refHalf[rt1],       rH2 = refHalf[rt2];

    // incident face (most anti-aligned with n on Inc)
    int   incFace = 0;
    float worstDot = 1.0f;
    for (int i = 0; i < 3; i++) {
        float d = incAxes[i].dot(n);
        if (d < worstDot) { worstDot = d; incFace = i; }
    }
    Vec3  incFaceNormal = (incAxes[incFace].dot(n) < 0.0f)
                        ?  incAxes[incFace]
                        : -incAxes[incFace];
    Vec3  incFaceCenter = Inc.position + incFaceNormal * incHalf[incFace];
    int   it1 = (incFace + 1) % 3,  it2 = (incFace + 2) % 3;
    Vec3  iT1 = incAxes[it1],       iT2 = incAxes[it2];
    float iH1 = incHalf[it1],       iH2 = incHalf[it2];

    // incident face polygon
    std::vector<Vec3> poly = {
        incFaceCenter + iT1*iH1 + iT2*iH2,
        incFaceCenter - iT1*iH1 + iT2*iH2,
        incFaceCenter - iT1*iH1 - iT2*iH2,
        incFaceCenter + iT1*iH1 - iT2*iH2
    };

    // clip against 4 side planes of reference face
    poly = satClip(poly,  rT1,  rT1.dot(refFaceCenter) + rH1); if (poly.empty()) return;
    poly = satClip(poly, -rT1, -rT1.dot(refFaceCenter) + rH1); if (poly.empty()) return;
    poly = satClip(poly,  rT2,  rT2.dot(refFaceCenter) + rH2); if (poly.empty()) return;
    poly = satClip(poly, -rT2, -rT2.dot(refFaceCenter) + rH2); if (poly.empty()) return;

    // depth test against reference face plane
    // keep points that are behind the reference face (penetrating)
    float refD = refFaceNormal.dot(refFaceCenter);
    m.contacts.clear();

    for (auto& p : poly) {
        float depth = refD - refFaceNormal.dot(p);
        if (depth > -0.001f) {
            ContactPoint cp;
            cp.penetrationDepth = std::max(depth, 0.0f);
            cp.position         = p + refFaceNormal * cp.penetrationDepth;
            m.contacts.push_back(cp);
        }
    }

    // keep 4 deepest
    if ((int)m.contacts.size() > 4) {
        std::sort(m.contacts.begin(), m.contacts.end(),
            [](const ContactPoint& a, const ContactPoint& b) {
                return a.penetrationDepth > b.penetrationDepth;
            });
        m.contacts.resize(4);
    }

    if (m.contacts.empty()) return;

    Vec3  avg      = {};
    float maxDepth = 0.0f;
    for (auto& cp : m.contacts) {
        avg      += cp.position;
        maxDepth  = std::max(maxDepth, cp.penetrationDepth);
    }
    m.contactPoint     = avg * (1.0f / (float)m.contacts.size());
    m.penetrationDepth = maxDepth;
}

// SAT box-box test
// returns false if separated
// fills outContact with normal, depth, and contact points
inline bool SATBoxBox(const RigidBody& A, const RigidBody& B,
                       ContactManifold& outContact)
{
    Vec3  axesA[3] = { A.orientation.rotate({1,0,0}),
                       A.orientation.rotate({0,1,0}),
                       A.orientation.rotate({0,0,1}) };
    float halfA[3] = { A.collider.halfExtents.x,
                       A.collider.halfExtents.y,
                       A.collider.halfExtents.z };

    Vec3  axesB[3] = { B.orientation.rotate({1,0,0}),
                       B.orientation.rotate({0,1,0}),
                       B.orientation.rotate({0,0,1}) };
    float halfB[3] = { B.collider.halfExtents.x,
                       B.collider.halfExtents.y,
                       B.collider.halfExtents.z };

    // 15 SAT axes (3 from A, 3 from B, 9 edge cross products)
    struct Candidate { Vec3 axis; float overlap; int type; }; // type 0=A face, 1=B face, 2=edge
    Candidate best = { {}, std::numeric_limits<float>::max(), -1 };

    auto test = [&](const Vec3& axis, int type) -> bool {
        float ov = axisOverlap(A.position, axesA, halfA,
                               B.position, axesB, halfB, axis);
        if (ov < 0.0f) return false; // separated
        if (ov < best.overlap) {
            best.overlap = ov;
            best.type    = type;
            // orient axis so it points from A to B
            Vec3 d = B.position - A.position;
            best.axis = (axis.dot(d) >= 0.0f)
                      ? axis.normalized()
                      : (-axis).normalized();
        }
        return true;
    };

    // face axes of A
    for (int i = 0; i < 3; i++)
        if (!test(axesA[i], 0)) return false;

    // face axes of B
    for (int i = 0; i < 3; i++)
        if (!test(axesB[i], 1)) return false;

    // edge cross products (9 axes)
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            if (!test(axesA[i].cross(axesB[j]), 2)) return false;

    // all axes overlap so collision is confirmed
    // best.axis points from A toward B, best.overlap is penetration depth
    outContact.normal           =  best.axis; // A->B
    outContact.penetrationDepth =  best.overlap;

    // determine reference body (the one whose face normal best matches collision normal)
    // face axes give stabler contact points than edge axes
    int refBody = 0; // default A
    if (best.type == 1) refBody = 1;
    else if (best.type == 2) {
        // edge to edge contact (pick whichever face is more aligned)
        float dotA = 0.0f, dotB = 0.0f;
        for (int i = 0; i < 3; i++) {
            dotA = std::max(dotA, std::abs(axesA[i].dot(best.axis)));
            dotB = std::max(dotB, std::abs(axesB[i].dot(best.axis)));
        }
        refBody = (dotA >= dotB) ? 0 : 1;
    }

    // flip normal convention (contact normal should point from B toward A)
    // (our resolver expects n = B->A)
    outContact.normal = -outContact.normal;

    generateSATContacts(outContact, A, B, best.axis, refBody);

    return !outContact.contacts.empty();
}


