#pragma once
#include "ContactManifold.h"
#include "physics/RigidBody.h"
#include <cmath>
#include <algorithm>

// impulse-based contact resolution for a single contact point
inline void resolveContact(ContactManifold& contact, float dt) {
    RigidBody* A = contact.bodyA;
    RigidBody* B = contact.bodyB;

    if (!A || !B) return;
    if (A->isStatic() && B->isStatic()) return;

    // ensure normal always points from B toward A
    Vec3 aToB = B->position - A->position;
    if (contact.normal.dot(aToB) > 0.0f)
        contact.normal = -contact.normal;

    Vec3 n  = contact.normal;
    Vec3 cp = contact.contactPoint;

    // vectors from centers of mass to contact point
    Vec3 rA = cp - A->position;
    Vec3 rB = cp - B->position;

    // relative velocity at contact point
    Vec3 vA  = A->velocityAtPoint(cp);
    Vec3 vB  = B->velocityAtPoint(cp);
    Vec3 vRel = vA - vB;

    float vRelN = vRel.dot(n);

    if (vRelN > 0.0f) return;

    // positional correction to prevent sinking due to numerical errors (Baumgarte stabilization)
    const float SLOP       = 0.02f;  // penetration allowance
    const float BAUMGARTE  = 0.15f;   // correction strength (0..1)

    float correction = std::max(contact.penetrationDepth - SLOP, 0.0f)
                     * BAUMGARTE
                     / (A->invMass + B->invMass);

    Vec3 correctionVec = n * correction;
    if (!A->isStatic()) A->position += correctionVec * A->invMass;
    if (!B->isStatic()) B->position -= correctionVec * B->invMass;

    // normal impulse (Coulomb model)
    float e = std::min(A->restitution, B->restitution);

    // angular inertia terms
    // (r * n) · (I⁻¹ (r * n))
    Vec3  rAxN    = rA.cross(n);
    Vec3  rBxN    = rB.cross(n);
    float angTermA = rAxN.dot(A->applyWorldInvInertia(rAxN));
    float angTermB = rBxN.dot(B->applyWorldInvInertia(rBxN));

    float denom = A->invMass + B->invMass + angTermA + angTermB;
    if (denom < 1e-10f) return;

    float j = -(1.0f + e) * vRelN / denom;

    // clamp impulse to prevent over-correction
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

    // friction impulse (Coulomb model)
    vA   = A->velocityAtPoint(cp);
    vB   = B->velocityAtPoint(cp);
    vRel = vA - vB;

    // tangential relative velocity (subtract normal component)
    Vec3 vRelTangent = vRel - n * vRel.dot(n);
    float tangentLen = vRelTangent.length();

    if (tangentLen < 1e-6f) return;

    Vec3 tangent = vRelTangent / tangentLen; // unit tangent direction

    // friction impulse magnitude
    Vec3  rAxT    = rA.cross(tangent);
    Vec3  rBxT    = rB.cross(tangent);
    float angFricA = rAxT.dot(A->applyWorldInvInertia(rAxT));
    float angFricB = rBxT.dot(B->applyWorldInvInertia(rBxT));

    float denomFric = A->invMass + B->invMass + angFricA + angFricB;
    if (denomFric < 1e-10f) return;

    float jFric = -tangentLen / denomFric;

    // Coulomb friction coefficient (average of the two bodies)
    float mu = (A->friction + B->friction) * 0.5f;

    // static vs dynamic friction
    Vec3 frictionImpulse;
    if (std::abs(jFric) <= j * mu) {
        // static friction - prevents sliding, scaled by actual required impulse
        frictionImpulse = tangent * jFric;
    } else {
        // Dynamic friction - opposes motion with maximum magnitude of μ * normal impulse
        frictionImpulse = tangent * (-j * mu);
    }

    if (!A->isStatic()) {
        A->linearVelocity  += frictionImpulse * A->invMass;
        A->angularVelocity += A->applyWorldInvInertia(rA.cross(frictionImpulse));
    }
    if (!B->isStatic()) {
        B->linearVelocity  -= frictionImpulse * B->invMass;
        B->angularVelocity -= B->applyWorldInvInertia(rB.cross(frictionImpulse));
    }

    // rolling friction
    if (!A->isStatic() && A->linearVelocity.lengthSq() < 0.1f) {
        float rollFric = 0.02f;
        A->angularVelocity *= (1.0f - rollFric);
        if (A->angularVelocity.lengthSq() < 0.001f)
            A->angularVelocity = {};
    }
    if (!B->isStatic() && B->linearVelocity.lengthSq() < 0.1f) {
        float rollFric = 0.02f;
        B->angularVelocity *= (1.0f - rollFric);
        if (B->angularVelocity.lengthSq() < 0.001f)
            B->angularVelocity = {};
    }
}

// resolve all contacts in the world (iterative solver)
inline void resolveAllContacts(std::vector<ContactManifold>& contacts,
                               float dt,
                               int iterations = 8)
{
    for (int i = 0; i < iterations; i++)
        for (auto& c : contacts)
            resolveContact(c, dt);
}


