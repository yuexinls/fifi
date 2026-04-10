#pragma once
#include "RigidBody.h"
#include <vector>
#include <memory>

class PhysicsWorld {
public:
    Vec3 gravity = {0, -9.81f, 0};
    float groundY = -2.0f; // simple infinite ground plane

    std::vector<std::unique_ptr<RigidBody>> bodies;

    RigidBody* addBody(RigidBody rb) {
        bodies.push_back(std::make_unique<RigidBody>(std::move(rb)));
        return bodies.back().get();
    }

    void step(float dt) {
        applyGravity();
        integrateBodies(dt);
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

    // temporary collision response with the ground plane 
    void resolveGroundPlane() {
        for (auto& body : bodies) {
            if (body->isStatic()) continue;

            float halfHeight = body->scale.y * 0.5f;
            float bottom = body->position.y + halfHeight;

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
            }
        }
    }
};


