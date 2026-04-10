#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <vector>

struct Vertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec2 uv;
};

class Mesh {
public:
    Mesh(const std::vector<Vertex>& vertices,
         const std::vector<uint32_t>& indices)
        : m_indexCount((GLsizei)indices.size())
    {
        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vbo);
        glGenBuffers(1, &m_ebo);

        glBindVertexArray(m_vao);

        // upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER, 
                     vertices.size() * sizeof(Vertex), 
                     vertices.data(), GL_STATIC_DRAW);
        
        // upload index data
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, 
                     indices.size() * sizeof(uint32_t), 
                     indices.data(), GL_STATIC_DRAW);

        // pos
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 
                              sizeof(Vertex), (void*)offsetof(Vertex, pos));
        // normal
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 
                              sizeof(Vertex), (void*)offsetof(Vertex, normal));
        // uv
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 
                              sizeof(Vertex), (void*)offsetof(Vertex, uv));

        glBindVertexArray(0);
    }

    ~Mesh() {
        glDeleteVertexArrays(1, &m_vao);
        glDeleteBuffers(1, &m_vbo);
        glDeleteBuffers(1, &m_ebo);
    }

    void draw() const {
        glBindVertexArray(m_vao);
        glDrawElements(GL_TRIANGLES, m_indexCount, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

    // cube mesh centered at origin
    static Mesh createCube() {
        std::vector<Vertex> vertices = {
            // pos                  normal     uv
            // front
            {{-0.5f,-0.5f, 0.5f}, { 0, 0, 1}, {0,0}},
            {{ 0.5f,-0.5f, 0.5f}, { 0, 0, 1}, {1,0}},
            {{ 0.5f, 0.5f, 0.5f}, { 0, 0, 1}, {1,1}},
            {{-0.5f, 0.5f, 0.5f}, { 0, 0, 1}, {0,1}},
            // back
            {{ 0.5f,-0.5f,-0.5f}, { 0, 0,-1}, {0,0}},
            {{-0.5f,-0.5f,-0.5f}, { 0, 0,-1}, {1,0}},
            {{-0.5f, 0.5f,-0.5f}, { 0, 0,-1}, {1,1}},
            {{ 0.5f, 0.5f,-0.5f}, { 0, 0,-1}, {0,1}},
            // left
            {{-0.5f,-0.5f,-0.5f}, {-1, 0, 0}, {0,0}},
            {{-0.5f,-0.5f, 0.5f}, {-1, 0, 0}, {1,0}},
            {{-0.5f, 0.5f, 0.5f}, {-1, 0, 0}, {1,1}},
            {{-0.5f, 0.5f,-0.5f}, {-1, 0, 0}, {0,1}},
            // right
            {{ 0.5f,-0.5f, 0.5f}, { 1, 0, 0}, {0,0}},
            {{ 0.5f,-0.5f,-0.5f}, { 1, 0, 0}, {1,0}},
            {{ 0.5f, 0.5f,-0.5f}, { 1, 0, 0}, {1,1}},
            {{ 0.5f, 0.5f, 0.5f}, { 1, 0, 0}, {0,1}},
            // top
            {{-0.5f, 0.5f, 0.5f}, { 0, 1, 0}, {0,0}},
            {{ 0.5f, 0.5f, 0.5f}, { 0, 1, 0}, {1,0}},
            {{ 0.5f, 0.5f,-0.5f}, { 0, 1, 0}, {1,1}},
            {{-0.5f, 0.5f,-0.5f}, { 0, 1, 0}, {0,1}},
            // bottom
            {{-0.5f,-0.5f,-0.5f}, { 0,-1, 0}, {0,0}},
            {{ 0.5f,-0.5f,-0.5f}, { 0,-1, 0}, {1,0}},
            {{ 0.5f,-0.5f, 0.5f}, { 0,-1, 0}, {1,1}},
            {{-0.5f,-0.5f, 0.5f}, { 0,-1, 0}, {0,1}},
        };

        std::vector<uint32_t> idx;
        for (uint32_t f = 0; f < 6; f++) {
            uint32_t b = f * 4;
            idx.insert(idx.end(), {b,b+1,b+2, b,b+2,b+3});
        }

        return Mesh(vertices, idx);
    }

    static Mesh createSphere(int stacks = 16, int slices = 16) {
        std::vector<Vertex> verts;
        std::vector<uint32_t> idx;

        for (int i = 0; i <= stacks; i++) {
            float phi = glm::pi<float>() * i / stacks; // 0 .. PI
            for (int j = 0; j <= slices; j++) {
                float theta = 2.0f * glm::pi<float>() * j / slices; // 0 .. 2PI

                glm::vec3 pos = {
                    std::sin(phi) * std::cos(theta),  // x
                    std::cos(phi),                    // y
                    std::sin(phi) * std::sin(theta)   // z
                };

                verts.push_back({
                    pos * 0.5f,
                    glm::normalize(pos),
                    { (float)j / slices,
                    (float)i / stacks }
                });
            }
        }

        for (int i = 0; i < stacks; i++) {
            for (int j = 0; j < slices; j++) {
                uint32_t a = i * (slices+1) + j;
                uint32_t b = a + slices + 1;
                idx.insert(idx.end(), { a, b, a+1,
                                        b, b+1, a+1 });
            }
        }

        return Mesh(verts, idx);
    }

private:
    GLuint m_vao, m_vbo, m_ebo;
    GLsizei m_indexCount;
    
};


