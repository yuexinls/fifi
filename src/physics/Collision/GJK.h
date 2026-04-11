#pragma once
#include "Collider.h"
#include "ContactManifold.h"
#include <array>
#include <vector>
#include <limits>
#include <cassert>

// GJK - Gilbert-Johnson-Keerthi algorithm
//
// Minkowski difference:
//   A ⊖ B = { a − b ∣ a ∈ A, b ∈ B }
//
// support function:
//
//   support_diff(d) = support_A(d) - support_B(-d)
//

struct GJKSupport {
    Vec3 point; // point on the Minkowski difference
    Vec3 pointA; // witness point on shape A
    Vec3 pointB; // witness point on shape B
};

inline GJKSupport minkowskiSupport(
    const Collider& A, const Vec3& posA, const Quaternion& rotA,
    const Collider& B, const Vec3& posB, const Quaternion& rotB,
    const Vec3& dir)
{
    Vec3 pA = A.support( dir, posA, rotA);
    Vec3 pB = B.support(-dir, posB, rotB);
    return { pA - pB, pA, pB };
}

// simplex in 3D can have at most 4 points (tetrahedron)

struct Simplex {
    std::array<GJKSupport, 4> pts;
    int size = 0;

    void push(const GJKSupport& s) {
        pts[3] = pts[2]; pts[2] = pts[1]; pts[1] = pts[0]; pts[0] = s;
        size   = std::min(size + 1, 4);
    }

    const GJKSupport& a() const { return pts[0]; }
    const GJKSupport& b() const { return pts[1]; }
    const GJKSupport& c() const { return pts[2]; }
    const GJKSupport& d() const { return pts[3]; }
};

// simplex subroutines

inline Vec3 tripleProduct(const Vec3& a, const Vec3& b, const Vec3& c) {
    // (a * b) * c = b(a·c) - a(b·c)
    return b * a.dot(c) - a * b.dot(c);
}

// returns true if the origin is in the same half space as toward relative to from

inline bool sameDir(const Vec3& v, const Vec3& toward) {
    return v.dot(toward) > 0.0f;
}

static bool doSimplex(Simplex& s, Vec3& dir);

static bool lineCase(Simplex& s, Vec3& dir) {
    Vec3 AB = s.b().point - s.a().point;
    Vec3 AO = -s.a().point;

    if (sameDir(AB, AO)) {
        dir = tripleProduct(AB, AO, AB);
    } else {
        s.size = 1;
        dir = AO;
    }
    return false;
}

static bool triangleCase(Simplex& s, Vec3& dir) {
    Vec3 AB = s.b().point - s.a().point;
    Vec3 AC = s.c().point - s.a().point;
    Vec3 AO = -s.a().point;

    Vec3 ABC = AB.cross(AC); // triangle normal

    if (sameDir(ABC.cross(AC), AO)) {
        if (sameDir(AC, AO)) {
            s.pts[1] = s.pts[2]; // keep A,C
            s.size = 2;
            dir = tripleProduct(AC, AO, AC);
        } else {
            s.size = 2;
            return lineCase(s, dir);
        }
    } else if (sameDir(AB.cross(ABC), AO)) {
        s.size = 2;
        return lineCase(s, dir);
    } else {
        // the origin is above or below the triangle
        if (sameDir(ABC, AO)) {
            dir = ABC;
        } else {
            // flip winding
            std::swap(s.pts[1], s.pts[2]);
            dir = -ABC;
        }
    }
    return false;
}

static bool tetrahedronCase(Simplex& s, Vec3& dir) {
    Vec3 AB = s.b().point - s.a().point;
    Vec3 AC = s.c().point - s.a().point;
    Vec3 AD = s.d().point - s.a().point;
    Vec3 AO = -s.a().point;

    Vec3 ABC = AB.cross(AC);
    Vec3 ACD = AC.cross(AD);
    Vec3 ADB = AD.cross(AB);

    if (sameDir(ABC, AO)) {
        s.size = 3;
        return triangleCase(s, dir);
    }
    if (sameDir(ACD, AO)) {
        s.pts[1] = s.pts[2];
        s.pts[2] = s.pts[3];
        s.size = 3;
        return triangleCase(s, dir);
    }

    if (sameDir(ADB, AO)) {
        s.pts[2] = s.pts[1];
        s.pts[1] = s.pts[3];
        s.size = 3;
        return triangleCase(s, dir);
    }

    return true; // origin is inside the tetrahedron
}

static bool doSimplex(Simplex& s, Vec3& dir) {
    switch (s.size) {
        case 2: return lineCase       (s, dir);
        case 3: return triangleCase   (s, dir);
        case 4: return tetrahedronCase(s, dir);
        default: assert(false && "invalid simplex size"); return false;
    }
    return false;
}

// EPA - Expanding Polytope Algorithm for penetration depth and contact points

struct EPAFace {
    Vec3 normal;
    float distance; // from origin along normal
    int i0, i1, i2; // indices of the simplex points that form this face
};

static EPAFace makeFace(const std::vector<GJKSupport>& pts,
                        int i0, int i1, int i2)
{
    Vec3 a = pts[i0].point;
    Vec3 b = pts[i1].point;
    Vec3 c = pts[i2].point;

    Vec3  n   = (b - a).cross(c - a).normalized();
    float d   = n.dot(a);

    if (d < 0) { n = -n; d = -d; }
    return { n, d, i0, i1, i2 };
}

static ContactManifold EPA(
    Simplex& simplex,
    const Collider& A, const Vec3& posA, const Quaternion& rotA,
    const Collider& B, const Vec3& posB, const Quaternion& rotB)
{
    std::vector<GJKSupport> pts = {
        simplex.a(), simplex.b(), simplex.c(), simplex.d()
    };

    std::vector<EPAFace> faces = {
        makeFace(pts, 0,1,2),
        makeFace(pts, 0,3,1),
        makeFace(pts, 0,2,3),
        makeFace(pts, 1,3,2)
    };

    const int   MAX_ITER  = 32;
    const float TOLERANCE = 0.0001f;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        // find the face closest to origin
        int   minIdx  = 0;
        float minDist = std::numeric_limits<float>::max();
        for (int i = 0; i < (int)faces.size(); i++) {
            if (faces[i].distance < minDist) {
                minDist = faces[i].distance;
                minIdx  = i;
            }
        }

        EPAFace& closest = faces[minIdx];

        // get support point in face normal direction
        GJKSupport sup = minkowskiSupport(
            A, posA, rotA, B, posB, rotB, closest.normal);

        float newDist = closest.normal.dot(sup.point);

        if (std::abs(newDist - minDist) < TOLERANCE) {
            // convergence - return contact manifold
            ContactManifold m;
            m.normal           = closest.normal;
            m.penetrationDepth = closest.distance;

            // approximate contact point as the average of the triangle vertices projected onto the face normal
            Vec3 pA = (pts[closest.i0].pointA +
                       pts[closest.i1].pointA +
                       pts[closest.i2].pointA) * (1.0f/3.0f);
            Vec3 pB = (pts[closest.i0].pointB +
                       pts[closest.i1].pointB +
                       pts[closest.i2].pointB) * (1.0f/3.0f);
            m.contactPoint = (pA + pB) * 0.5f;
            return m;
        }

        // add the new point to the polytope and rebuild faces
        int newIdx = (int)pts.size();
        pts.push_back(sup);

        std::vector<std::pair<int,int>> edges;
        std::vector<EPAFace> newFaces;

        for (auto& f : faces) {
            if (f.normal.dot(sup.point - pts[f.i0].point) > 0) {
                // face is visible from the new point, remove it and add its edges to the edge list
                auto addEdge = [&](int a, int b) {
                    for (auto it = edges.begin(); it != edges.end(); ++it) {
                        if (it->first == b && it->second == a) {
                            edges.erase(it); return;
                        }
                    }
                    edges.push_back({a, b});
                };
                addEdge(f.i0, f.i1);
                addEdge(f.i1, f.i2);
                addEdge(f.i2, f.i0);
            } else {
                newFaces.push_back(f);
            }
        }

        for (auto& [e0, e1] : edges)
            newFaces.push_back(makeFace(pts, e0, e1, newIdx));

        faces = std::move(newFaces);
    }

    return {}; // did not converge
}

// public entry point for GJK + EPA collision detection

inline bool GJKIntersect(
    const Collider& A, const Vec3& posA, const Quaternion& rotA,
    const Collider& B, const Vec3& posB, const Quaternion& rotB,
    ContactManifold& outContact)
{
    Vec3 dir = (posB - posA);
    if (dir.lengthSq() < 1e-10f) dir = { 1, 0, 0 };

    Simplex s;
    s.push(minkowskiSupport(A, posA, rotA, B, posB, rotB, dir));
    dir = -s.a().point;

    const int MAX_ITER = 64;
    for (int i = 0; i < MAX_ITER; i++) {
        if (dir.lengthSq() < 1e-10f) break;

        GJKSupport sup = minkowskiSupport(A, posA, rotA, B, posB, rotB, dir);

        if (sup.point.dot(dir) < 0.0f)
            return false; // no intersection

        s.push(sup);

        if (doSimplex(s, dir)) {
            // intersection detected, now run EPA to find contact manifold
            outContact = EPA(s, A, posA, rotA, B, posB, rotB);
            return true;
        }
    }
    return false;
}


