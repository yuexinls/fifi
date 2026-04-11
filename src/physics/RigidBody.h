#pragma once
#include "math/Vec3.h"
#include "math/Mat4.h"
#include "math/Quaternion.h"
#include "Collision/Collider.h"

enum class ShapeType {
    Box,
    Sphere
};

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
    ShapeType shape = ShapeType::Box;

    Collider collider;

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
        rb.shape    = ShapeType::Box;

        rb.collider.type        = Collider::Type::Box;
        rb.collider.halfExtents = halfExtents;

        // I = (1/12) * m * (h² + d²) for each axis
        float wx = halfExtents.x * 2.0f;
        float wy = halfExtents.y * 2.0f;
        float wz = halfExtents.z * 2.0f;

        float ix = (1.0f/12.0f) * m * (wy*wy + wz*wz);
        float iy = (1.0f/12.0f) * m * (wx*wx + wz*wz);
        float iz = (1.0f/12.0f) * m * (wx*wx + wy*wy);
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
        rb.shape    = ShapeType::Sphere;

        rb.collider.type   = Collider::Type::Sphere;
        rb.collider.radius = radius;

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
        rb.invInertiaTensorLocal = {0.0f, 0.0f, 0.0f};
        rb.scale    = halfExtents * 2.0f;
        rb.color    = {0.5f, 0.5f, 0.5f};
        rb.shape    = ShapeType::Box;

        rb.collider.type        = Collider::Type::Box;
        rb.collider.halfExtents = halfExtents;

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
        linearVelocity *= std::exp(-linearDamping * dt);
        position       += linearVelocity * dt;

        // angular
        Vec3 angularAccel = applyWorldInvInertia(m_torqueAccum);
        angularVelocity  += angularAccel * dt;

        // angular damping, mimics rotational friction
        angularVelocity *= std::exp(-angularDamping * dt);

        // integrate orientation via quaternion derivative
        orientation = orientation.integrateAngularVelocity(angularVelocity, dt);
        orientation.normalize();

        const float LINEAR_SLEEP  = 0.01f;
        const float ANGULAR_SLEEP = 0.01f;

        if (linearVelocity.lengthSq()  < LINEAR_SLEEP  * LINEAR_SLEEP)
            linearVelocity  = {};
        if (angularVelocity.lengthSq() < ANGULAR_SLEEP * ANGULAR_SLEEP)
            angularVelocity = {};

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
    float linearDamping = 0.8f;
    float angularDamping = 3.0f;

private:
    Vec3 m_forceAccum;
    Vec3 m_torqueAccum;
};


