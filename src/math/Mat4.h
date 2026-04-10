#pragma once
#include "Vec3.h"
#include "Quaternion.h"
#include <glm/glm.hpp>
#include <cmath>
#include <cstring>

// column-major 4x4 matrix
// m[col][row] — col 0..3, row 0..3
struct Mat4 {
    float m[4][4];

    Mat4() { setIdentity(); }

    void setIdentity() {
        memset(m, 0, sizeof(m));
        m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0f;
    }

    static Mat4 identity() { return {}; }

    static Mat4 zero() {
        Mat4 r; memset(r.m, 0, sizeof(r.m)); return r;
    }

    // matrix multiplication
    Mat4 operator*(const Mat4& r) const {
        Mat4 out = zero();
        for (int col = 0; col < 4; col++)
            for (int row = 0; row < 4; row++)
                for (int k   = 0; k   < 4; k++)
                    out.m[col][row] += m[k][row] * r.m[col][k];
        return out;
    }

    // transform a direction (w=0)
    Vec3 transformDir(const Vec3& v) const {
        return {
            m[0][0]*v.x + m[1][0]*v.y + m[2][0]*v.z,
            m[0][1]*v.x + m[1][1]*v.y + m[2][1]*v.z,
            m[0][2]*v.x + m[1][2]*v.y + m[2][2]*v.z
        };
    }

    // transform a point (w=1)
    Vec3 transformPoint(const Vec3& v) const {
        return {
            m[0][0]*v.x + m[1][0]*v.y + m[2][0]*v.z + m[3][0],
            m[0][1]*v.x + m[1][1]*v.y + m[2][1]*v.z + m[3][1],
            m[0][2]*v.x + m[1][2]*v.y + m[2][2]*v.z + m[3][2]
        };
    }

    // transform functions

    static Mat4 translation(const Vec3& t) {
        Mat4 r;
        r.m[3][0] = t.x;
        r.m[3][1] = t.y;
        r.m[3][2] = t.z;
        return r;
    }

    static Mat4 scale(const Vec3& s) {
        Mat4 r;
        r.m[0][0] = s.x;
        r.m[1][1] = s.y;
        r.m[2][2] = s.z;
        return r;
    }

    // converts a unit quaternion to a rotation matrix
    static Mat4 rotation(const Quaternion& q) {
        float x=q.x, y=q.y, z=q.z, w=q.w;
        Mat4 r;
        r.m[0][0] = 1 - 2*(y*y + z*z);
        r.m[0][1] =     2*(x*y + z*w);
        r.m[0][2] =     2*(x*z - y*w);

        r.m[1][0] =     2*(x*y - z*w);
        r.m[1][1] = 1 - 2*(x*x + z*z);
        r.m[1][2] =     2*(y*z + x*w);

        r.m[2][0] =     2*(x*z + y*w);
        r.m[2][1] =     2*(y*z - x*w);
        r.m[2][2] = 1 - 2*(x*x + y*y);

        r.m[3][3] = 1.0f;
        return r;
    }

    // model matrix from position + rotation + scale
    static Mat4 trs(const Vec3& pos, const Quaternion& rot, const Vec3& s) {
        return translation(pos) * rotation(rot) * scale(s);
    }

    // projection matrix from vertical field of view, aspect ratio etc

    static Mat4 perspective(float fovY, float aspect, float zNear, float zFar) {
        float tanHalf = std::tan(fovY * 0.5f);
        Mat4 r = zero();
        r.m[0][0] =  1.0f / (aspect * tanHalf);
        r.m[1][1] =  1.0f / tanHalf;
        r.m[2][2] = -(zFar + zNear) / (zFar - zNear);
        r.m[2][3] = -1.0f;
        r.m[3][2] = -(2.0f * zFar * zNear) / (zFar - zNear);
        return r;
    }

    static Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
        Vec3 f = (target - eye).normalized(); // forward
        Vec3 r = f.cross(up).normalized();    // right
        Vec3 u = r.cross(f);                  // true up

        Mat4 out;
        out.m[0][0] =  r.x; out.m[1][0] =  r.y; out.m[2][0] =  r.z;
        out.m[0][1] =  u.x; out.m[1][1] =  u.y; out.m[2][1] =  u.z;
        out.m[0][2] = -f.x; out.m[1][2] = -f.y; out.m[2][2] = -f.z;
        out.m[3][0] = -r.dot(eye);
        out.m[3][1] = -u.dot(eye);
        out.m[3][2] =  f.dot(eye);
        return out;
    }

    // 3x3 inverse-transpose for transforming normals
    Mat4 normalMatrix() const {
        float a = m[0][0], b = m[1][0], c = m[2][0];
        float d = m[0][1], e = m[1][1], f = m[2][1];
        float g = m[0][2], h = m[1][2], i = m[2][2];

        float det = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g);
        if (std::abs(det) < 1e-10f) return identity();

        Mat4 r = zero();
        r.m[0][0] = (e*i-f*h)/det;  r.m[1][0] = -(b*i-c*h)/det; r.m[2][0] = (b*f-c*e)/det;
        r.m[0][1] = -(d*i-f*g)/det; r.m[1][1] =  (a*i-c*g)/det; r.m[2][1] = -(a*f-c*d)/det;
        r.m[0][2] = (d*h-e*g)/det;  r.m[1][2] = -(a*h-b*g)/det; r.m[2][2] = (a*e-b*d)/det;
        r.m[3][3] = 1.0f;
        return r;
    }

    // converts to a glm mat4 for passing to openGL shaders
    glm::mat4 toGlm() const {
        glm::mat4 r;
        memcpy(&r[0][0], &m[0][0], 16 * sizeof(float));
        return r;
    }
};


