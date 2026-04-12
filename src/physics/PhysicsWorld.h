#pragma once
#include "RigidBody.h"
#include "Collision/BroadPhase.h"
#include "Collision/GJK.h"
#include "Collision/ContactManifold.h"
#include "Collision/ContactResolver.h"
#include "Collision/ContactGenerator.h"
#include "Collision/SAT.h"
#include "PhysicsWatchdog.h"
#include <vector>
#include <memory>

class PhysicsWorld {
public:
    Vec3 gravity = {0, -9.81f, 0};
    float groundY = -2.0f; // simple infinite ground plane

    PhysicsWatchdog watchdog;
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
        watchdog.analyse(bodies, contacts);
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

    static bool sphereBoxCollide(const RigidBody& sphere,
                                 const RigidBody& box,
                                 ContactManifold& m)
    {
        // transform sphere center into box local space
        Vec3 localCenter = box.orientation.conjugate().rotate(
            sphere.position - box.position);

        Vec3 he = box.collider.halfExtents;
        float r = sphere.collider.radius;

        // closest point on box to sphere center (in local space)
        Vec3 closest = {
            std::clamp(localCenter.x, -he.x, he.x),
            std::clamp(localCenter.y, -he.y, he.y),
            std::clamp(localCenter.z, -he.z, he.z)
        };

        Vec3  diff   = localCenter - closest;
        float distSq = diff.lengthSq();

        if (distSq >= r * r) return false;

        float dist = std::sqrt(distSq);

        Vec3 localNormal;
        float penetration;

        if (dist < 1e-6f) {
            float overlaps[6] = {
                he.x - localCenter.x,   // +x face
                he.x + localCenter.x,   // -x face
                he.y - localCenter.y,   // +y face
                he.y + localCenter.y,   // -y face
                he.z - localCenter.z,   // +z face
                he.z + localCenter.z    // -z face
            };
            Vec3 normals[6] = {
                {1,0,0}, {-1,0,0},
                {0,1,0}, {0,-1,0},
                {0,0,1}, {0,0,-1}
            };

            int   best    = 0;
            float minOver = overlaps[0];
            for (int k = 1; k < 6; k++) {
                if (overlaps[k] < minOver) { minOver = overlaps[k]; best = k; }
            }

            localNormal = normals[best];
            penetration = r + minOver;   // full sphere radius + face overlap
        } else {
            localNormal = diff * (1.0f / dist);   // points box-surface -> sphere-center
            penetration = r - dist;
        }

        Vec3 worldNormal  = box.orientation.rotate(localNormal);  // box→sphere direction
        Vec3 worldClosest = box.position + box.orientation.rotate(closest);

        // Normal convention: this function always returns "box → sphere" direction.
        // The caller is responsible for flipping if needed to satisfy resolver B→A.
        m.normal           = worldNormal;
        m.penetrationDepth = penetration;
        m.contactPoint     = worldClosest;

        ContactPoint cp;
        cp.position         = worldClosest;
        cp.penetrationDepth = penetration;
        m.contacts          = { cp };

        return true;
    }

    void detectCollisions() {
        contacts.clear();
        broadphasePairs = broadphase(bodies);

        for (auto& [i, j] : broadphasePairs) {

            auto& bi = bodies[i];
            auto& bj = bodies[j];

            bool aBox = (bi->collider.type == Collider::Type::Box);
            bool bBox = (bj->collider.type == Collider::Type::Box);
            bool aSphere = !aBox;
            bool bSphere = !bBox;

            ContactManifold m;
            bool hit = false;

            if (aBox && bBox) {
                hit = SATBoxBox(*bi, *bj, m);

            } else if (aSphere && bBox) {
                hit = sphereBoxCollide(*bi, *bj, m);

            } else if (aBox && bSphere) {
                hit = sphereBoxCollide(*bj, *bi, m);
                if (hit) m.normal = -m.normal;

            } else {
                // sphere-sphere via GJK
                hit = GJKIntersect(bi->collider, bi->position, bi->orientation,
                                   bj->collider, bj->position, bj->orientation, m);
                if (hit) {
                    ContactPoint cp;
                    cp.position         = m.contactPoint;
                    cp.penetrationDepth = m.penetrationDepth;
                    m.contacts          = { cp };
                }
            }

            if (!hit) continue;
            if (m.contacts.empty()) continue;
            if (m.penetrationDepth <= 0.0f || m.penetrationDepth > 2.0f) continue;
            if (m.normal.lengthSq() < 0.9f) continue;

            m.bodyA = bi.get();
            m.bodyB = bj.get();

            if (m.valid()) contacts.push_back(m);
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


