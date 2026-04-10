#pragma once
#include "math/Vec3.h"
#include "math/Mat4.h"
#include "math/Quaternion.h"

class RigidBody {
public:
    // primary state
    Vec3       position;
    Vec3       linearVelocity;
    Quaternion orientation;
    Vec3       angularVelocity;

    // mass properties
    float mass = 1.0f;
    float invMass = 1.0f; // 0 is infinite mass (static)

    Vec3 invInertiaTensorLocal = {1,1,1}; // diagonal inertia tensor in local space

    // material properties
    float restitution = 0.4f; // bounciness
    float friction    = 0.5f; // surface friction

    // rendering hints
    Vec3 color = { 0.4f, 0.6f, 1.0f };
    Vec3 scale = { 1.0f, 1.0f, 1.0f }; // visual only

    // constructor helpers
    static RigidBody createBox(float m,
                               const Vec3& halfExtents,
                               const Vec3& pos = {})
    {
        RigidBody rb;
        rb.position = pos;
        rb.mass     = m;
        rb.invMass  = (m > 0.0f) ? 1.0f / m : 0.0f;
        rb.scale    = halfExtents * 2.0f;

        // I = (1/12) * m * (h² + d²) for each axis
        float ex = halfExtents.x, ey = halfExtents.y, ez = halfExtents.z;
        float ix = (1.0f/12.0f) * m * (ey*ey + ez*ez) * 4;
        float iy = (1.0f/12.0f) * m * (ex*ex + ez*ez) * 4;
        float iz = (1.0f/12.0f) * m * (ex*ex + ey*ey) * 4;
        rb.invInertiaTensorLocal = {
            (ix > 0) ? 1.0f/ix : 0.0f,
            (iy > 0) ? 1.0f/iy : 0.0f,
            (iz > 0) ? 1.0f/iz : 0.0f
        };
        return rb;
    }

    static RigidBody createSphere(float m, float radius, const Vec3& pos = {}) {
        RigidBody rb;
        rb.position = pos;
        rb.mass     = m;
        rb.invMass  = (m > 0.0f) ? 1.0f / m : 0.0f;
        rb.scale    = { radius*2, radius*2, radius*2 };

        // I = (2/5) * m * r² for a solid sphere
        float i = (2.0f/5.0f) * m * radius * radius;
        float inv = (i > 0) ? 1.0f / i : 0.0f;
        rb.invInertiaTensorLocal = { inv, inv, inv };
        return rb;
    }

    // static
    static RigidBody createStatic(const Vec3& pos,
                                  const Vec3& halfExtents)
    {
        RigidBody rb;
        rb.position = pos;
        rb.mass     = 0.0f;
        rb.invMass  = 0.0f;
        rb.invInertiaTensorLocal = {};
        rb.scale    = halfExtents * 2.0f;
        rb.color    = {0.5f, 0.5f, 0.5f};
        return rb;
    }

    // force / torque accumulationuh

    void applyForce(const Vec3& f) {
        m_forceAccum += f;
    }

    void applyForceAtPoint(const Vec3& f, const Vec3& worldPoint) {
        m_forceAccum  += f;
        Vec3 r         = worldPoint - position;
        m_torqueAccum += r.cross(f);
    }

    void applyTorque(const Vec3& t) {
        m_torqueAccum += t;
    }

    void clearAccumulators() {
        m_forceAccum = {};
        m_torqueAccum = {};
    }

    // intertia tensor in world space (recomputed each tick)

    Vec3 getWorldInvInertia(const Vec3& localInvI) const {
        // convert orientation quaternion to rotation matrix
        Vec3 c0 = orientation.rotate({1,0,0});
        Vec3 c1 = orientation.rotate({0,1,0});
        Vec3 c2 = orientation.rotate({0,0,1});

        // R * diag(I_inv) * R^T  applied to a vector is \/
        // sum over axes:  (R_col_i · v) * localInvI[i] * R_col_i
        (void)localInvI; (void)c0; (void)c1; (void)c2;
        return localInvI;
    }

    // I_world_inv * v  =  R * (I_local_inv * (R^T * v))
    Vec3 applyWorldInvInertia(const Vec3& v) const {
        Vec3 local = orientation.conjugate().rotate(v);
        Vec3 scaled = {
            local.x * invInertiaTensorLocal.x,
            local.y * invInertiaTensorLocal.y,
            local.z * invInertiaTensorLocal.z
        };
        return orientation.rotate(scaled);
    }

    // integration step (symplectic Euler)

    void integrate(float dt) {
        if (invMass <= 0.0f) {
            clearAccumulators();
            return; // static body
        }

        // linear
        Vec3 linearAccel = m_forceAccum * invMass;
        linearVelocity += linearAccel * dt;
        
        // linear damping, mimics air resistance
        linearVelocity *= std::pow(1.0f - linearDamping, dt);
        position       += linearVelocity * dt;

        // angular
        Vec3 angularAccel = applyWorldInvInertia(m_torqueAccum);
        angularVelocity  += angularAccel * dt;

        // angular damping, mimics rotational friction
        angularVelocity *= std::pow(1.0f - angularDamping, dt);

        // integrate orientation via quaternion derivative
        orientation = orientation.integrateAngularVelocity(angularVelocity, dt);
        orientation.normalize();

        clearAccumulators();
    }

    // Helpers

    // build the model matrix for rendering
    Mat4 modelMatrix() const {
        return Mat4::trs(position, orientation, scale);
    }

    // velocity of a point on the body given by worldPoint
    Vec3 velocityAtPoint(const Vec3& worldPoint) const {
        Vec3 r = worldPoint - position;
        return linearVelocity + angularVelocity.cross(r);
    }

    bool isStatic() const { return invMass <= 0.0f; }

    // damping coefficients (0..1) / second
    float linearDamping = 0.05f;
    float angularDamping = 0.1f;

private:
    Vec3 m_forceAccum;
    Vec3 m_torqueAccum;
};


