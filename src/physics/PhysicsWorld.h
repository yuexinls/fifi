#pragma once
#include "RigidBody.h"
#include "Collision/BroadPhase.h"
#include "Collision/GJK.h"
#include "Collision/ContactManifold.h"
#include "Collision/ContactResolver.h"
#include "Collision/ContactGenerator.h"
#include <vector>
#include <memory>

class PhysicsWorld {
public:
    Vec3 gravity = {0, -9.81f, 0};
    float groundY = -2.0f; // simple infinite ground plane

    std::vector<std::unique_ptr<RigidBody>> bodies;

    std::vector<ContactManifold> contacts;

    std::vector<std::pair<int,int>> broadphasePairs;

    RigidBody* addBody(RigidBody rb) {
        bodies.push_back(std::make_unique<RigidBody>(std::move(rb)));
        return bodies.back().get();
    }

    void step(float dt) {
        applyGravity();
        integrateBodies(dt);
        detectCollisions();
        resolveAllContacts(contacts, dt, 12);
        resolveGroundPlane(); // replaced later
    }

private:
    void applyGravity() {
        for (auto& body : bodies) {
            if (body->isStatic()) continue;
            body->applyForce(gravity * body->mass);
        }
    }

    void integrateBodies(float dt) {
        for (auto& body : bodies)
            body->integrate(dt);
    }

    void detectCollisions() {
        contacts.clear();
        broadphasePairs = broadphase(bodies);

        for (auto& [i, j] : broadphasePairs) {
            // floor plane is handled separately so ig we can skip it here
            if (i == 0 || j == 0) continue;

            auto& bi = bodies[i];
            auto& bj = bodies[j];

            ContactManifold m;
            bool hit = GJKIntersect(
                bi->collider, bi->position, bi->orientation,
                bj->collider, bj->position, bj->orientation,
                m);

            if (!hit) continue;

            if (m.penetrationDepth <= 0.0f
            || m.penetrationDepth >  5.0f
            || m.normal.lengthSq() < 0.9f)
                continue;

            m.bodyA = bi.get();
            m.bodyB = bj.get();
            enrichManifold(m, *bi, *bj);

            if (m.valid())
                contacts.push_back(m);
        }
    }

    // temporary collision response with the ground plane 
    void resolveGroundPlane() {
        for (auto& body : bodies) {
            if (body->isStatic()) continue;

            float halfHeight = body->scale.y * 0.5f;
            float bottom = body->position.y - halfHeight;

            if (bottom < groundY) {
                body->position.y = groundY + halfHeight;

                if (body->linearVelocity.y < 0) {
                    body->linearVelocity.y *= -body->restitution;
                    
                    // kill tiny bounces
                    if (std::abs(body->linearVelocity.y) < 0.05f)
                        body->linearVelocity.y = 0.0f;
                }

                // simple friction on horizontal velocity
                body->linearVelocity.x *= (1.0f - body->friction * 0.1f);
                body->linearVelocity.z *= (1.0f - body->friction * 0.1f);

                if (body->linearVelocity.lengthSq() < 0.1f) {
                    body->angularVelocity *= 0.98f;
                    if (body->angularVelocity.lengthSq() < 0.001f)
                        body->angularVelocity = {};
                }
            }
        }
    }
};


