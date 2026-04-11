#pragma once
#include "math/Vec3.h"
#include "math/Quaternion.h"
#include <algorithm>

struct Collider {
    enum class Type {
        Sphere,
        Box
    };

    Type type        = Type::Box;
    Vec3 halfExtents = {0.5f, 0.5f, 0.5f}; // for box
    float radius     = 0.5f; // for sphere

    // AABB for broadphase collision detection
    struct AABB {
        Vec3 min, max;

        bool overlaps(const AABB& o) const {
            return (min.x <= o.max.x && max.x >= o.min.x) &&
                   (min.y <= o.max.y && max.y >= o.min.y) &&
                   (min.z <= o.max.z && max.z >= o.min.z);
        }

        Vec3 center()  const { return (min + max) * 0.5f; }
        Vec3 extents() const { return (max - min) * 0.5f; }
    };

    // build the AABB in world space
    AABB computeAABB(const Vec3& pos, const Quaternion& rot) const {
        if (type == Type::Sphere) {
            return { pos - Vec3(radius), pos + Vec3(radius) };
        }

        // for a box, project each axis of the OBB onto the world axes
        Vec3 ax = rot.rotate({ halfExtents.x, 0, 0 });
        Vec3 ay = rot.rotate({ 0, halfExtents.y, 0 });
        Vec3 az = rot.rotate({ 0, 0, halfExtents.z });

        Vec3 e = {
            std::abs(ax.x) + std::abs(ay.x) + std::abs(az.x),
            std::abs(ax.y) + std::abs(ay.y) + std::abs(az.y),
            std::abs(ax.z) + std::abs(ay.z) + std::abs(az.z)
        };

        return { pos - e, pos + e };
    }

    // support function for GJK
    Vec3 support(const Vec3& d,
                 const Vec3& pos,
                 const Quaternion& rot) const
    {
        if (type == Type::Sphere) {
            Vec3 dir = d;
            if (dir.lengthSq() < 1e-10f)
                dir = {1,0,0}; // arbitrary direction if d is zero

            return pos + dir.normalized() * radius;
        }

        // for a box, find the corner in the direction of d
        Vec3 dir = d;
        if (dir.lengthSq() < 1e-10f)
            dir = {1,0,0}; // again

        Vec3 localDir = rot.conjugate().rotate(dir);
        Vec3 corner = {
            (localDir.x >= 0 ? halfExtents.x : -halfExtents.x),
            (localDir.y >= 0 ? halfExtents.y : -halfExtents.y),
            (localDir.z >= 0 ? halfExtents.z : -halfExtents.z)
        };
        return pos + rot.rotate(corner);
    }
};


