#pragma once
#include "Collider.h"
#include "ContactManifold.h"
#include <vector>
#include <array>
#include <limits>
#include <algorithm>

struct SupportPoint {
    Vec3 point;   // point on Minkowski difference
    Vec3 pointA;  // witness on A
    Vec3 pointB;  // witness on B
};

static SupportPoint support(
    const Collider& A, const Vec3& posA, const Quaternion& rotA,
    const Collider& B, const Vec3& posB, const Quaternion& rotB,
    const Vec3& dir)
{
    Vec3 pA = A.support( dir, posA, rotA);
    Vec3 pB = B.support(-dir, posB, rotB);
    return { pA - pB, pA, pB };
}

// Simplex
struct Simplex {
    SupportPoint pts[4];
    int size = 0;

    void clear() { size = 0; }

    void push(const SupportPoint& p) {
        pts[3] = pts[2];
        pts[2] = pts[1];
        pts[1] = pts[0];
        pts[0] = p;
        size   = std::min(size + 1, 4);
    }

    void set(std::initializer_list<SupportPoint> list) {
        size = 0;
        for (auto& p : list) pts[size++] = p;
    }
};

// Helper
static inline bool sameDirection(const Vec3& a, const Vec3& b) {
    return a.dot(b) > 0.0f;
}

// Line case
static bool lineCase(Simplex& s, Vec3& dir) {
    const SupportPoint& A = s.pts[0];
    const SupportPoint& B = s.pts[1];

    Vec3 AB = B.point - A.point;
    Vec3 AO = -A.point;

    if (sameDirection(AB, AO)) {
        // Origin is between A and B
        // (AB * AO) * AB
        Vec3 cross = AB.cross(AO);
        if (cross.lengthSq() < 1e-10f) {
            return true;
        }
        dir = cross.cross(AB);
    } else {
        s.set({ A });
        dir = AO;
    }
    return false;
}

// Triangle case
static bool triangleCase(Simplex& s, Vec3& dir) {
    const SupportPoint& A = s.pts[0];
    const SupportPoint& B = s.pts[1];
    const SupportPoint& C = s.pts[2];

    Vec3 AB  = B.point - A.point;
    Vec3 AC  = C.point - A.point;
    Vec3 AO  = -A.point;
    Vec3 ABC = AB.cross(AC); // triangle normal

    if (sameDirection(ABC.cross(AC), AO)) {
        if (sameDirection(AC, AO)) {
            s.set({ A, C });
            dir = AC.cross(AO).cross(AC);
        } else {
            s.set({ A, B });
            return lineCase(s, dir);
        }
    } else if (sameDirection(AB.cross(ABC), AO)) {
        s.set({ A, B });
        return lineCase(s, dir);
    } else {
        // origin is above or below the triangle
        if (sameDirection(ABC, AO)) {
            // correct winding
            dir = ABC;
        } else {
            // flip winding so normal points toward origin
            s.set({ A, C, B });
            dir = -ABC;
        }
    }
    return false;
}

// Tetrahedron case
static bool tetrahedronCase(Simplex& s, Vec3& dir) {
    const SupportPoint& A = s.pts[0];
    const SupportPoint& B = s.pts[1];
    const SupportPoint& C = s.pts[2];
    const SupportPoint& D = s.pts[3];

    Vec3 AB = B.point - A.point;
    Vec3 AC = C.point - A.point;
    Vec3 AD = D.point - A.point;
    Vec3 AO = -A.point;

    Vec3 ABC = AB.cross(AC);
    Vec3 ACD = AC.cross(AD);
    Vec3 ADB = AD.cross(AB);

    // Check each face
    if (sameDirection(ABC, AO)) {
        s.set({ A, B, C });
        return triangleCase(s, dir);
    }
    if (sameDirection(ACD, AO)) {
        s.set({ A, C, D });
        return triangleCase(s, dir);
    }
    if (sameDirection(ADB, AO)) {
        s.set({ A, D, B });
        return triangleCase(s, dir);
    }

    // origin is inside tetrahedron
    return true;
}

static bool doSimplex(Simplex& s, Vec3& dir) {
    switch (s.size) {
        case 2: return lineCase      (s, dir);
        case 3: return triangleCase  (s, dir);
        case 4: return tetrahedronCase(s, dir);
        default: return false;
    }
}

// GJK entry point
static bool GJK(
    const Collider& A, const Vec3& posA, const Quaternion& rotA,
    const Collider& B, const Vec3& posB, const Quaternion& rotB,
    Simplex& outSimplex)
{
    Vec3 dir = posA - posB;
    if (dir.lengthSq() < 1e-10f) dir = { 1, 0, 0 };

    Simplex s;
    s.push(support(A, posA, rotA, B, posB, rotB, dir));

    dir = -s.pts[0].point;

    const int MAX_ITER = 64;
    for (int i = 0; i < MAX_ITER; i++) {
        if (dir.lengthSq() < 1e-10f) break;

        SupportPoint p = support(A, posA, rotA, B, posB, rotB, dir);

        if (p.point.dot(dir) < 0.0f)
            return false;

        s.push(p);

        if (doSimplex(s, dir)) {
            outSimplex = s;
            return true;
        }
    }
    return false;
}

// EPA - Expanding Polytope Algorithm
struct EPAFace {
    Vec3  normal;
    float dist;
    int   a, b, c;
};

static EPAFace epaFace(const std::vector<SupportPoint>& verts,
                       int a, int b, int c)
{
    Vec3 pa = verts[a].point;
    Vec3 pb = verts[b].point;
    Vec3 pc = verts[c].point;

    Vec3 cross = (pb - pa).cross(pc - pa);
    if (cross.lengthSq() < 1e-14f)
        return { {0,1,0}, 1e9f, a, b, c }; // degenerate

    Vec3  n = cross.normalized();
    float d = n.dot(pa);

    // ensure normal points away from origin
    if (d < 0) { n = -n; d = -d; }
    return { n, d, a, b, c };
}

static ContactManifold EPA(
    Simplex& simplex,
    const Collider& A, const Vec3& posA, const Quaternion& rotA,
    const Collider& B, const Vec3& posB, const Quaternion& rotB)
{
    // Seed polytope from tetrahedron
    std::vector<SupportPoint> verts = {
        simplex.pts[0], simplex.pts[1],
        simplex.pts[2], simplex.pts[3]
    };

    std::vector<EPAFace> faces = {
        epaFace(verts, 0,1,2),
        epaFace(verts, 0,3,1),
        epaFace(verts, 0,2,3),
        epaFace(verts, 1,3,2)
    };

    const int   MAX_ITER  = 64;
    const float TOLERANCE = 1e-4f;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        if (faces.empty()) break;

        // find the face closest to the origin
        int   minIdx = 0;
        float minD   = faces[0].dist;
        for (int i = 1; i < (int)faces.size(); i++) {
            if (faces[i].dist < minD) {
                minD   = faces[i].dist;
                minIdx = i;
            }
        }

        EPAFace closest = faces[minIdx];
        SupportPoint sp = support(
            A, posA, rotA, B, posB, rotB, closest.normal);
        float newDist = closest.normal.dot(sp.point);

        // converged
        if (newDist - minD < TOLERANCE) {
            ContactManifold m;
            m.normal           =  closest.normal;
            m.penetrationDepth =  closest.dist;

            // average witness points for contact position
            Vec3 pA = (verts[closest.a].pointA
                     + verts[closest.b].pointA
                     + verts[closest.c].pointA) * (1.0f/3.0f);
            Vec3 pB = (verts[closest.a].pointB
                     + verts[closest.b].pointB
                     + verts[closest.c].pointB) * (1.0f/3.0f);
            m.contactPoint = (pA + pB) * 0.5f;
            return m;
        }

        // Expand polytope with new point
        int newIdx = (int)verts.size();
        verts.push_back(sp);

        std::vector<std::pair<int,int>> horizon;
        std::vector<EPAFace> newFaces;

        for (auto& f : faces) {
            if (f.normal.dot(sp.point - verts[f.a].point) > 0.0f) {
                auto addEdge = [&](int p, int q) {
                    for (auto it = horizon.begin(); it != horizon.end(); ++it) {
                        if (it->first == q && it->second == p) {
                            horizon.erase(it);
                            return;
                        }
                    }
                    horizon.push_back({p, q});
                };
                addEdge(f.a, f.b);
                addEdge(f.b, f.c);
                addEdge(f.c, f.a);
            } else {
                newFaces.push_back(f);
            }
        }

        // guard against degenerate cases where horizon is empty
        if (horizon.empty()) break;

        for (auto& [e0, e1] : horizon) {
            EPAFace nf = epaFace(verts, e0, e1, newIdx);
            if (nf.dist < 1e8f) // skip bad faces
                newFaces.push_back(nf);
        }

        if (newFaces.empty()) break;
        faces = std::move(newFaces);
    }

    return {}; // failed to converge
}

// Public entry point
inline bool GJKIntersect(
    const Collider& A, const Vec3& posA, const Quaternion& rotA,
    const Collider& B, const Vec3& posB, const Quaternion& rotB,
    ContactManifold& outContact)
{
    Simplex s;
    if (!GJK(A, posA, rotA, B, posB, rotB, s))
        return false;

    // make sure we have a full tetrahedron for EPA
    while (s.size < 4) {
        Vec3 dir = { 1, 0, 0 };
        if (s.size == 2) {
            Vec3 ab = s.pts[1].point - s.pts[0].point;
            dir = ab.cross({0,1,0});
            if (dir.lengthSq() < 1e-6f)
                dir = ab.cross({0,0,1});
        } else if (s.size == 3) {
            Vec3 ab = s.pts[1].point - s.pts[0].point;
            Vec3 ac = s.pts[2].point - s.pts[0].point;
            dir = ab.cross(ac);
        }
        if (dir.lengthSq() < 1e-10f) break;
        s.push(support(A, posA, rotA, B, posB, rotB, dir.normalized()));
        // avoid duplicate points
        bool dup = false;
        for (int i = 1; i < s.size; i++)
            if ((s.pts[0].point - s.pts[i].point).lengthSq() < 1e-10f)
                { dup = true; break; }
        if (dup) { s.size--; break; }
    }

    if (s.size < 4) return false;

    outContact = EPA(s, A, posA, rotA, B, posB, rotB);
    return outContact.penetrationDepth > 0.0f
        && outContact.normal.lengthSq() > 0.5f;
}


