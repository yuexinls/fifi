#pragma once
#include "ContactManifold.h"
#include "physics/RigidBody.h"
#include "core/DebugLog.h"
#include <cmath>
#include <algorithm>
#include <vector>
#include <sstream>
#include <iomanip>

// resolves only velocity
inline void resolveContactVelocity(ContactManifold& contact) {
    RigidBody* A = contact.bodyA;
    RigidBody* B = contact.bodyB;

    if (!A || !B) return;
    if (A->isStatic() && B->isStatic()) return;

    Vec3 n  = contact.normal;
    Vec3 cp = contact.contactPoint;

    Vec3 rA = cp - A->position;
    Vec3 rB = cp - B->position;

    Vec3 vA   = A->velocityAtPoint(cp);
    Vec3 vB   = B->velocityAtPoint(cp);
    Vec3 vRel = vA - vB;

    float vRelN = vRel.dot(n);

    // already separating or resting
    if (vRelN > 0.0f) return;

    float e = std::min(A->restitution, B->restitution);

    // kill resitution for very slow impacts to prevent jitter
    if (std::abs(vRelN) < 0.5f) e = 0.0f;

    Vec3  rAxN     = rA.cross(n);
    Vec3  rBxN     = rB.cross(n);
    float angTermA = rAxN.dot(A->applyWorldInvInertia(rAxN));
    float angTermB = rBxN.dot(B->applyWorldInvInertia(rBxN));
    float denom    = A->invMass + B->invMass + angTermA + angTermB;

    if (denom < 1e-10f) return;

    float j = -(1.0f + e) * vRelN / denom;
    j = std::max(j, 0.0f);

    Vec3 impulse = n * j;

    if (!A->isStatic()) {
        A->linearVelocity  += impulse * A->invMass;
        A->angularVelocity += A->applyWorldInvInertia(rA.cross(impulse));
    }
    if (!B->isStatic()) {
        B->linearVelocity  -= impulse * B->invMass;
        B->angularVelocity -= B->applyWorldInvInertia(rB.cross(impulse));
    }

    // friction impulse
    vA   = A->velocityAtPoint(cp);
    vB   = B->velocityAtPoint(cp);
    vRel = vA - vB;

    Vec3  vt        = vRel - n * vRel.dot(n);
    float vtLen     = vt.length();
    if (vtLen < 1e-6f) return;

    Vec3  tangent   = vt / vtLen;
    Vec3  rAxT      = rA.cross(tangent);
    Vec3  rBxT      = rB.cross(tangent);
    float angFricA  = rAxT.dot(A->applyWorldInvInertia(rAxT));
    float angFricB  = rBxT.dot(B->applyWorldInvInertia(rBxT));
    float denomFric = A->invMass + B->invMass + angFricA + angFricB;

    if (denomFric < 1e-10f) return;

    float jFric  = -vtLen / denomFric;
    float mu     = (A->friction + B->friction) * 0.5f;

    Vec3 frictionImpulse = (std::abs(jFric) <= j * mu)
        ? tangent * jFric
        : tangent * (-j * mu);

    if (!A->isStatic()) {
        A->linearVelocity  += frictionImpulse * A->invMass;
        A->angularVelocity += A->applyWorldInvInertia(rA.cross(frictionImpulse));
    }
    if (!B->isStatic()) {
        B->linearVelocity  -= frictionImpulse * B->invMass;
        B->angularVelocity -= B->applyWorldInvInertia(rB.cross(frictionImpulse));
    }

    // rolling friction / damping for very slow contacts to prevent jitter
    if (!A->isStatic() && A->linearVelocity.lengthSq() < 0.05f) {
        A->angularVelocity *= 0.98f;
        if (A->angularVelocity.lengthSq() < 0.001f) A->angularVelocity = {};
    }
    if (!B->isStatic() && B->linearVelocity.lengthSq() < 0.05f) {
        B->angularVelocity *= 0.98f;
        if (B->angularVelocity.lengthSq() < 0.001f) B->angularVelocity = {};
    }
}

// positional correction to prevent sinking and jitter
inline void resolveContactPosition(ContactManifold& contact) {
    RigidBody* A = contact.bodyA;
    RigidBody* B = contact.bodyB;

    if (!A || !B) return;
    if (A->isStatic() && B->isStatic()) return;

    const float SLOP      = 0.02f;
    const float BAUMGARTE = 0.11111115f;
    const float MAX_CORRECTION = 0.02f;

    float penetration = contact.penetrationDepth - SLOP;
    if (penetration <= 0.0f) return;

    float totalInvMass = A->invMass + B->invMass;
    if (totalInvMass < 1e-10f) return;

    float correctionMag = std::min(
        penetration * BAUMGARTE / totalInvMass,
        MAX_CORRECTION);

    Vec3 correction = contact.normal * correctionMag;

    if (!A->isStatic()) A->position += correction * A->invMass;
    if (!B->isStatic()) B->position -= correction * B->invMass;
}

inline void resolveContactVelocityAtPoint(RigidBody* A, RigidBody* B,
                                           const Vec3& cp, const Vec3& n,
                                           float penetration,
                                           float scale = 1.0f)
{
    if (!A || !B) return;

    Vec3 rA = cp - A->position;
    Vec3 rB = cp - B->position;

    Vec3 vA   = A->velocityAtPoint(cp);
    Vec3 vB   = B->velocityAtPoint(cp);
    Vec3 vRel = vA - vB;

    float vRelN = vRel.dot(n);
    if (vRelN > 0.0f) return;

    float e = std::min(A->restitution, B->restitution);
    if (std::abs(vRelN) < 0.5f) e = 0.0f;

    Vec3  rAxN     = rA.cross(n);
    Vec3  rBxN     = rB.cross(n);
    float angTermA = rAxN.dot(A->applyWorldInvInertia(rAxN));
    float angTermB = rBxN.dot(B->applyWorldInvInertia(rBxN));
    float denom    = A->invMass + B->invMass + angTermA + angTermB;
    if (denom < 1e-10f) return;

    // scale impulse by penetration depth for better stacking stability
    float j = -(1.0f + e) * vRelN / denom;
    j = std::max(j, 0.0f);
    j *= scale;

    // log large impulses
    if (j > WARN_IMPULSE) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "Large impulse j=" << j
        << " vRelN=" << vRelN
        << " denom=" << denom
        << " scale=" << scale;
        DWARN("large_impulse", ss.str());
    }

    Vec3 impulse = n * j;

    if (!A->isStatic()) {
        A->linearVelocity  += impulse * A->invMass;
        A->angularVelocity += A->applyWorldInvInertia(rA.cross(impulse));
    }
    if (!B->isStatic()) {
        B->linearVelocity  -= impulse * B->invMass;
        B->angularVelocity -= B->applyWorldInvInertia(rB.cross(impulse));
    }

    // friction impulse
    vA   = A->velocityAtPoint(cp);
    vB   = B->velocityAtPoint(cp);
    vRel = vA - vB;

    Vec3  vt    = vRel - n * vRel.dot(n);
    float vtLen = vt.length();
    if (vtLen < 1e-6f) return;

    Vec3  tangent   = vt / vtLen;
    Vec3  rAxT      = rA.cross(tangent);
    Vec3  rBxT      = rB.cross(tangent);
    float angFricA  = rAxT.dot(A->applyWorldInvInertia(rAxT));
    float angFricB  = rBxT.dot(B->applyWorldInvInertia(rBxT));
    float denomFric = A->invMass + B->invMass + angFricA + angFricB;
    if (denomFric < 1e-10f) return;

    float mu    = (A->friction + B->friction) * 0.5f;
    float jFric = -vtLen / denomFric;
    jFric *= scale;

    Vec3 frictionImpulse = (std::abs(jFric) <= j * mu)
        ? tangent * jFric
        : tangent * (-j * mu);

    if (!A->isStatic()) {
        A->linearVelocity  += frictionImpulse * A->invMass;
        A->angularVelocity += A->applyWorldInvInertia(rA.cross(frictionImpulse));
    }
    if (!B->isStatic()) {
        B->linearVelocity  -= frictionImpulse * B->invMass;
        B->angularVelocity -= B->applyWorldInvInertia(rB.cross(frictionImpulse));
    }
}

// iteratively resolve a list of contacts, applying positional correction at the end
inline void resolveAllContacts(std::vector<ContactManifold>& contacts,
                               float dt,
                               int iterations = 10)
{
    for (int iter = 0; iter < iterations; iter++) {
        for (auto& c : contacts) {
            if (!c.valid()) continue;

            // each contact manifold may have multiple contact points
            float pointScale = 1.0f / (float)c.contacts.size();

            for (auto& cp : c.contacts) {
                resolveContactVelocityAtPoint(
                    c.bodyA, c.bodyB,
                    cp.position, c.normal,
                    cp.penetrationDepth,
                    pointScale);
            }
        }
    }

    for (auto& c : contacts) {
        if (!c.valid()) continue;
        resolveContactPosition(c);
    }
}


