#pragma once
#include "math/Vec3.h"

struct RigidBody; // forward declaration

struct ContactManifold {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Vec3 normal;            // from B to A
    float penetrationDepth = 0.0f;
    Vec3 contactPoint;      // in world space

    bool valid() const { return bodyA && bodyB && penetrationDepth > 1e-5f && normal.lengthSq() > 1e-6f; }
};


