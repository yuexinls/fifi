#pragma once
#include "Collider.h"
#include <vector>
#include <utility>
#include <memory>
#include "RigidBody.h"

// returns all pairs of body indices whose AABBs overlap
// O(n²) - fine up to ~500 bodies
// for larger scenes, i'll eventually replace
// with a sweep-and-prune or BVH

inline std::vector<std::pair<int,int>> broadphase(
    const std::vector<std::unique_ptr<RigidBody>>& bodies)
{
    std::vector<std::pair<int,int>> pairs;
    pairs.reserve(bodies.size());

    std::vector<Collider::AABB> aabbs(bodies.size());
    for (size_t i = 0; i < bodies.size(); i++) {
        aabbs[i] = bodies[i]->collider.computeAABB(
            bodies[i]->position, bodies[i]->orientation);
    }

    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            if (bodies[i]->isStatic() && bodies[j]->isStatic()) continue;

            if (aabbs[i].overlaps(aabbs[j]))
                pairs.emplace_back((int)i, (int)j);
        }
    }

    return pairs;
}


