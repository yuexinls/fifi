#pragma once
#include <glm/glm.hpp>
#include <cmath>
#include <ostream>

struct Vec3 {
    float x, y, z;

    Vec3()                         : x(0), y(0), z(0) {}
    Vec3(float x, float y, float z): x(x), y(y), z(z) {}
    explicit Vec3(float s)         : x(s), y(s), z(s) {}

    // arithmetic operators
    Vec3 operator+(const Vec3& r) const { return {x+r.x, y+r.y, z+r.z}; }
    Vec3 operator-(const Vec3& r) const { return {x-r.x, y-r.y, z-r.z}; }
    Vec3 operator*(float s)       const { return {x*s,   y*s,   z*s  }; }
    Vec3 operator/(float s)       const { return {x/s,   y/s,   z/s  }; }
    Vec3 operator-()              const { return {-x,    -y,    -z   }; }

    Vec3& operator+=(const Vec3& r) { x+=r.x; y+=r.y; z+=r.z; return *this; }
    Vec3& operator-=(const Vec3& r) { x-=r.x; y-=r.y; z-=r.z; return *this; }
    Vec3& operator*=(float s)       { x*=s;   y*=s;   z*=s;   return *this; }
    Vec3& operator/=(float s)       { x/=s;   y/=s;   z/=s;   return *this; }

    bool operator==(const Vec3& r) const { return x==r.x && y==r.y && z==r.z; }
    bool operator!=(const Vec3& r) const { return !(*this == r); }

    // core operations
    // dot product
    float dot(const Vec3& r) const { return x*r.x + y*r.y + z*r.z; }

    // cross product
    Vec3 cross(const Vec3& r) const {
        return {
            y * r.z - z * r.y,
            z * r.x - x * r.z,
            x * r.y - y * r.x
        };
    }

    float lengthSq() const { return x*x + y*y + z*z; }
    float length()   const { return std::sqrt(lengthSq()); }

    // normalize in-place
    Vec3 normalized() const {
        float len = length();
        if (len < 1e-10f) return {0,0,0};
        return *this / len;
    }

    void normalize() { *this = normalized(); }

    // reflect this vector around a normal
    Vec3 reflect(const Vec3& normal) const {
        return *this - normal * (2.0f * dot(normal));
    }

    // linear interpolation aka lerp
    static Vec3 lerp(const Vec3& a, const Vec3& b, float t) {
        return a + (b - a) * t;
    }

    // component-wise min/max
    static Vec3 min(const Vec3& a, const Vec3& b) {
        return { std::fmin(a.x, b.x), std::fmin(a.y, b.y), std::fmin(a.z, b.z) };
    }
    static Vec3 max(const Vec3& a, const Vec3& b) {
        return { std::fmax(a.x, b.x), std::fmax(a.y, b.y), std::fmax(a.z, b.z) };
    }

    // converts to glm for passing to shaders
    glm::vec3 toGlm() const { return {x, y, z}; }

    friend Vec3 operator*(float s, const Vec3& v) { return v * s; }
    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
};


