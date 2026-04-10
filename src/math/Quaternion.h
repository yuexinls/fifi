#pragma once
#include <glm/gtc/quaternion.hpp>
#include "Vec3.h"
#include <cmath>

// quarternion: q = w + xi + yj + zk
struct Quaternion {
    float w, x, y, z;

    Quaternion()                                   : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    // create a quaternion from an axis and angle (in radians)
    static Quaternion fromAxisAngle(const Vec3& axis, float angle) {
        float halfAngle = angle * 0.5f;
        float s         = std::sin(halfAngle);
        return {
            std::cos(halfAngle),
            axis.x * s,
            axis.y * s,
            axis.z * s
        };
    }

    // build from euler angles (radians) applied in zyx order
    static Quaternion fromEuler(float pitch, float yaw, float roll) {
        float cp = std::cos(pitch*0.5f), sp = std::sin(pitch*0.5f);
        float cy = std::cos(yaw  *0.5f), sy = std::sin(yaw  *0.5f);
        float cr = std::cos(roll *0.5f), sr = std::sin(roll *0.5f);
        return {
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy
        };
    }

    // hamilton product (combining rotations)
    Quaternion operator*(const Quaternion& r) const {
        return {
            w*r.w - x*r.x - y*r.y - z*r.z,
            w*r.x + x*r.w + y*r.z - z*r.y,
            w*r.y - x*r.z + y*r.w + z*r.x,
            w*r.z + x*r.y - y*r.x + z*r.w
        };
    }

    Quaternion operator*=(const Quaternion& r) { *this = *this * r; return *this; }

    // conjugate (inverse for unit quaternions)
    Quaternion conjugate() const { return {w, -x, -y, -z}; }

    float normSq() const { return w*w + x*x + y*y + z*z; }
    float norm()   const { return std::sqrt(normSq()); }

    Quaternion normalized() const {
        float n = norm();
        if (n < 1e-10f) return {};
        return {w/n, x/n, y/n, z/n};
    }

    void normalize() { *this = normalized(); }

    // rotate a vector by this quaternion
    // uses the sandwich product: q * (0,v) * q^-1
    Vec3 rotate(const Vec3& v) const {
        Vec3 qv = {x, y, z};
        Vec3 uv = qv.cross(v);
        Vec3 uuv = qv.cross(uv);
        return v + (uv * w + uuv) * 2.0f;
    }

    // spherical linear interpolation between two quaternions
    static Quaternion slerp(Quaternion a, Quaternion b, float t) {
        float dot = a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;

        // the shorter path
        if (dot < 0.0f) { b.w=-b.w; b.x=-b.x; b.y=-b.y; b.z=-b.z; dot=-dot; }

        // fall back to linear interpolation if very close
        if (dot > 0.9995f) {
            Quaternion r = {
                a.w + t*(b.w-a.w), a.x + t*(b.x-a.x),
                a.y + t*(b.y-a.y), a.z + t*(b.z-a.z)
            };
            return r.normalized();
        }

        float theta0 = std::acos(dot);
        float theta  = theta0 * t;
        float s0     = std::cos(theta) - dot * std::sin(theta) / std::sin(theta0);
        float s1     = std::sin(theta) / std::sin(theta0);

        return {
            a.w*s0 + b.w*s1, a.x*s0 + b.x*s1,
            a.y*s0 + b.y*s1, a.z*s0 + b.z*s1
        };
    }

    // integrate angular velocity (rad/s) over timestep dt
    Quaternion integrateAngularVelocity(const Vec3& omega, float dt) const {
        float angle = omega.length() * dt;
        if (angle < 1e-10f) return *this;
        Vec3 axis = omega.normalized();
        return (*this * fromAxisAngle(axis, angle)).normalized();
    }

    glm::quat toGlm() const { return {w, x, y, z}; }
};


