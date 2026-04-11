#pragma once
#include "Collider.h"
#include <vector>
#include <utility>
#include "RigidBody.h"

// returns all pairs of body indices whose AABBs overlap
// O(n²) - fine up to ~500 bodies
// for larger scenes, i'll eventually replace
// with a sweep-and-prune or BVH

inline std::vector<std::pair<int,int>> broadphase(
    const std::vector<std::unique_ptr<RigidBody>>& bodies)
{
    std::vector<std::pair<int,int>> pairs;

    for (int i = 0; i < (int)bodies.size(); i++) {
        auto& bi = bodies[i];
        Collider::AABB aabbI = bi->collider.computeAABB(
            bi->position, bi->orientation);

        for (int j = i + 1; j < (int)bodies.size(); j++) {
            auto& bj = bodies[j];

            if (bi->isStatic() && bj->isStatic()) continue;

            Collider::AABB aabbJ = bj->collider.computeAABB(
                bj->position, bj->orientation);

            if (aabbI.overlaps(aabbJ))
                pairs.push_back({ i, j });
        }
    }

    return pairs;
}


