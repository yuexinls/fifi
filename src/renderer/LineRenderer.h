#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <vector>

// batches line segments and renders them in one draw call

class LineRenderer {
public:
    LineRenderer() {
        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vbo);
        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glEnableVertexAttribArray(0); // position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1); // color
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                              6 * sizeof(float), (void*)(3*sizeof(float)));
        glBindVertexArray(0);
    }

    ~LineRenderer() {
        glDeleteVertexArrays(1, &m_vao);
        glDeleteBuffers(1, &m_vbo);
    }

    void begin() { m_verts.clear(); }

    void addLine(const glm::vec3& a, const glm::vec3& b, const glm::vec3& col) {
        m_verts.insert(m_verts.end(), { a.x,a.y,a.z, col.r,col.g,col.b });
        m_verts.insert(m_verts.end(), { b.x,b.y,b.z, col.r,col.g,col.b });
    }

    // draw an AABB with the given min/max corners and color
    void addAABB(const glm::vec3& mn, const glm::vec3& mx,
                 const glm::vec3& col)
    {
        // bottom face
        addLine({mn.x,mn.y,mn.z},{mx.x,mn.y,mn.z}, col);
        addLine({mx.x,mn.y,mn.z},{mx.x,mn.y,mx.z}, col);
        addLine({mx.x,mn.y,mx.z},{mn.x,mn.y,mx.z}, col);
        addLine({mn.x,mn.y,mx.z},{mn.x,mn.y,mn.z}, col);
        // top face
        addLine({mn.x,mx.y,mn.z},{mx.x,mx.y,mn.z}, col);
        addLine({mx.x,mx.y,mn.z},{mx.x,mx.y,mx.z}, col);
        addLine({mx.x,mx.y,mx.z},{mn.x,mx.y,mx.z}, col);
        addLine({mn.x,mx.y,mx.z},{mn.x,mx.y,mn.z}, col);
        // verticals
        addLine({mn.x,mn.y,mn.z},{mn.x,mx.y,mn.z}, col);
        addLine({mx.x,mn.y,mn.z},{mx.x,mx.y,mn.z}, col);
        addLine({mx.x,mn.y,mx.z},{mx.x,mx.y,mx.z}, col);
        addLine({mn.x,mn.y,mx.z},{mn.x,mx.y,mx.z}, col);
    }

    void draw() {
        if (m_verts.empty()) return;
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     m_verts.size() * sizeof(float),
                     m_verts.data(), GL_DYNAMIC_DRAW);
        glBindVertexArray(m_vao);
        glDrawArrays(GL_LINES, 0, (GLsizei)(m_verts.size() / 6));
        glBindVertexArray(0);
    }

private:
    GLuint m_vao, m_vbo;
    std::vector<float> m_verts;
};


