#pragma once
#include "ContactManifold.h"
#include "physics/RigidBody.h"
#include "core/DebugLog.h"
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

// Erin Catto’s sequential impulse method resolves around clamping accumulated impulses to ensure stable
// and non explosive contact resolution across multiple points
struct ContactPointState {
    float accumulatedNormal   = 0.0f; // running total normal impulse
    float accumulatedFriction1 = 0.0f;
    float accumulatedFriction2 = 0.0f;
};

static void resolvePoint(RigidBody* A, RigidBody* B,
                          const Vec3& cp,
                          const Vec3& n,       // B->A
                          float       penetration,
                          ContactPointState& state)
{
    if (!A || !B) return;
    if (A->isStatic() && B->isStatic()) return;

    Vec3 rA = cp - A->position;
    Vec3 rB = cp - B->position;

    // relative velocity at contact point
    Vec3  vA   = A->velocityAtPoint(cp);
    Vec3  vB   = B->velocityAtPoint(cp);
    Vec3  vRel = vA - vB;
    float vRelN = vRel.dot(n);

    // already separating (no normal impulse needed)
    if (vRelN > 0.0f && state.accumulatedNormal == 0.0f) return;

    // effective mass along normal
    Vec3  rAxN    = rA.cross(n);
    Vec3  rBxN    = rB.cross(n);
    float angA    = rAxN.dot(A->applyWorldInvInertia(rAxN));
    float angB    = rBxN.dot(B->applyWorldInvInertia(rBxN));
    float effMass = A->invMass + B->invMass + angA + angB;
    if (effMass < 1e-10f) return;

    // restitution (zero for slow contacts to avoid micro bounce)
    float e = (std::abs(vRelN) > 1.0f)
            ? std::min(A->restitution, B->restitution)
            : 0.0f;

    // compute delta impulse and clamp accumulated total >= 0
    float dj  = -(1.0f + e) * vRelN / effMass;
    float oldAcc = state.accumulatedNormal;
    state.accumulatedNormal = std::max(oldAcc + dj, 0.0f);
    float applyJ = state.accumulatedNormal - oldAcc; // actual delta

    // apply normal impulse
    Vec3 impulse = n * applyJ;
    if (!A->isStatic()) {
        A->linearVelocity  += impulse * A->invMass;
        A->angularVelocity += A->applyWorldInvInertia(rA.cross(impulse));
    }
    if (!B->isStatic()) {
        B->linearVelocity  -= impulse * B->invMass;
        B->angularVelocity -= B->applyWorldInvInertia(rB.cross(impulse));
    }

    // friction (two tangent axes [stable frame, not velocity aligned])
    // using a fixed tangent frame avoids direction flip jitter
    Vec3 t1, t2;
    if (std::abs(n.x) < 0.57f) {
        t1 = n.cross({1,0,0}).normalized();
    } else {
        t1 = n.cross({0,1,0}).normalized();
    }
    t2 = n.cross(t1).normalized();

    // re read velocity after normal impulse
    vA   = A->velocityAtPoint(cp);
    vB   = B->velocityAtPoint(cp);
    vRel = vA - vB;

    float mu = (A->friction + B->friction) * 0.5f;
    float maxFric = mu * state.accumulatedNormal;

    // tangent 1
    float vRelT1 = vRel.dot(t1);
    {
        Vec3  rAxT1  = rA.cross(t1);
        Vec3  rBxT1  = rB.cross(t1);
        float angA1  = rAxT1.dot(A->applyWorldInvInertia(rAxT1));
        float angB1  = rBxT1.dot(B->applyWorldInvInertia(rBxT1));
        float em1    = A->invMass + B->invMass + angA1 + angB1;
        if (em1 > 1e-10f) {
            float df1    = -vRelT1 / em1;
            float oldF1  = state.accumulatedFriction1;
            state.accumulatedFriction1 = std::clamp(oldF1 + df1, -maxFric, maxFric);
            float applyF1 = state.accumulatedFriction1 - oldF1;
            Vec3  fi1     = t1 * applyF1;
            if (!A->isStatic()) {
                A->linearVelocity  += fi1 * A->invMass;
                A->angularVelocity += A->applyWorldInvInertia(rA.cross(fi1));
            }
            if (!B->isStatic()) {
                B->linearVelocity  -= fi1 * B->invMass;
                B->angularVelocity -= B->applyWorldInvInertia(rB.cross(fi1));
            }
        }
    }

    // tangent 2
    vA   = A->velocityAtPoint(cp);
    vB   = B->velocityAtPoint(cp);
    vRel = vA - vB;
    float vRelT2 = vRel.dot(t2);
    {
        Vec3  rAxT2  = rA.cross(t2);
        Vec3  rBxT2  = rB.cross(t2);
        float angA2  = rAxT2.dot(A->applyWorldInvInertia(rAxT2));
        float angB2  = rBxT2.dot(B->applyWorldInvInertia(rBxT2));
        float em2    = A->invMass + B->invMass + angA2 + angB2;
        if (em2 > 1e-10f) {
            float df2    = -vRelT2 / em2;
            float oldF2  = state.accumulatedFriction2;
            state.accumulatedFriction2 = std::clamp(oldF2 + df2, -maxFric, maxFric);
            float applyF2 = state.accumulatedFriction2 - oldF2;
            Vec3  fi2     = t2 * applyF2;
            if (!A->isStatic()) {
                A->linearVelocity  += fi2 * A->invMass;
                A->angularVelocity += A->applyWorldInvInertia(rA.cross(fi2));
            }
            if (!B->isStatic()) {
                B->linearVelocity  -= fi2 * B->invMass;
                B->angularVelocity -= B->applyWorldInvInertia(rB.cross(fi2));
            }
        }
    }

    // rolling friction for contacts nearly resting
    if (!A->isStatic() && A->linearVelocity.lengthSq() < 0.05f) {
        A->angularVelocity *= 0.98f;
        if (A->angularVelocity.lengthSq() < 0.001f) A->angularVelocity = {};
    }
    if (!B->isStatic() && B->linearVelocity.lengthSq() < 0.05f) {
        B->angularVelocity *= 0.98f;
        if (B->angularVelocity.lengthSq() < 0.001f) B->angularVelocity = {};
    }
}

// positional correction (split impulse [once per step, after velocity])
static void resolvePosition(ContactManifold& contact) {
    RigidBody* A = contact.bodyA;
    RigidBody* B = contact.bodyB;
    if (!A || !B) return;
    if (A->isStatic() && B->isStatic()) return;

    const float SLOP       = 0.005f;
    const float BAUMGARTE  = 0.35f;
    const float MAX_CORR   = 0.08f;

    float pen = contact.penetrationDepth - SLOP;
    if (pen <= 0.0f) return;

    float totalInvMass = A->invMass + B->invMass;
    if (totalInvMass < 1e-10f) return;

    float mag = std::min(pen * BAUMGARTE / totalInvMass, MAX_CORR);
    Vec3  corr = contact.normal * mag;

    if (!A->isStatic()) A->position += corr * A->invMass;
    if (!B->isStatic()) B->position -= corr * B->invMass;
}

// main solver (persistent state per contact point across iterations)
inline void resolveAllContacts(std::vector<ContactManifold>& contacts,
                                float dt,
                                int iterations = 10)
{
    // allocate persistent state for each contact point
    // state persists across iterations but *not* across frames
    struct ManifoldState {
        std::vector<ContactPointState> points;
    };
    std::vector<ManifoldState> states(contacts.size());
    for (int i = 0; i < (int)contacts.size(); i++) {
        states[i].points.resize(contacts[i].contacts.size());
    }

    // velocity iterations
    for (int iter = 0; iter < iterations; iter++) {
        for (int ci = 0; ci < (int)contacts.size(); ci++) {
            auto& c = contacts[ci];
            if (!c.valid()) continue;

            for (int pi = 0; pi < (int)c.contacts.size(); pi++) {
                auto& cp    = c.contacts[pi];
                auto& state = states[ci].points[pi];
                resolvePoint(c.bodyA, c.bodyB,
                             cp.position, c.normal,
                             cp.penetrationDepth,
                             state);
            }
        }
    }

    // one positional correction pass
    for (auto& c : contacts) {
        if (c.valid())
            resolvePosition(c);
    }
}


