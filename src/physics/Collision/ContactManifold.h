#pragma once
#include "math/Vec3.h"
#include <vector>

struct RigidBody;

struct ContactPoint {
    Vec3  position;
    float penetrationDepth;
};

struct ContactManifold {
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;

    Vec3 normal;

    // multiple contact points for box-box collisions
    std::vector<ContactPoint> contacts;

    // convenience fields for resolution
    Vec3  contactPoint     = {};
    float penetrationDepth = 0.0f;

    bool valid() const {
        return bodyA && bodyB
            && penetrationDepth > 0.0f
            && normal.lengthSq() > 0.5f;
    }
};


