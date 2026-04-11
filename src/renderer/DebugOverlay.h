#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#define STB_EASY_FONT_IMPLEMENTATION
#include "stb_easy_font.h"

#include <string>
#include <vector>

class DebugOverlay {
public:
    bool visible = false;

    DebugOverlay() {
        const char* vert = R"(
            #version 450 core
            layout(location=0) in vec2 aPos;
            layout(location=1) in vec4 aColor;
            uniform vec2 uScreenSize;
            out vec4 vColor;
            void main() {
                // Convert pixel coords to NDC
                vec2 ndc = (aPos / uScreenSize) * 2.0 - 1.0;
                ndc.y = -ndc.y;
                gl_Position = vec4(ndc, 0.0, 1.0);
                vColor = aColor;
            }
        )";
        const char* frag = R"(
            #version 450 core
            in vec4 vColor;
            out vec4 FragColor;
            void main() { FragColor = vColor; }
        )";

        m_shader = compileShader(vert, frag);

        glGenVertexArrays(1, &m_vao);
        glGenBuffers(1, &m_vbo);
        glGenBuffers(1, &m_ebo);
        glBindVertexArray(m_vao);
        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);

        // stb_easy_font produces vertices with 2 floats for position and 4 for color
        glEnableVertexAttribArray(0); // position
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE,
                              16, (void*)0);
        glEnableVertexAttribArray(1); // color (4 unsigned bytes)
        glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE,
                              16, (void*)12);
        glBindVertexArray(0);
    }

    ~DebugOverlay() {
        glDeleteVertexArrays(1, &m_vao);
        glDeleteBuffers(1, &m_vbo);
        glDeleteBuffers(1, &m_ebo);
        glDeleteProgram(m_shader);
    }

    void onKey(int key, int action) {
        if (key == GLFW_KEY_F3 && action == GLFW_PRESS) {
            visible = !visible;
        }
    }

    // call after 3d scene is drawn
    void draw(int screenW, int screenH,
              float fps,
              int bodyCount,
              int broadphasePairs,
              int contactCount,
              const std::vector<std::string>& bodyLines)
    {
        if (!visible) return;

        // build all the text lines
        std::vector<std::string> lines;
        lines.push_back("FPS:              " + fmt(fps, 1));
        lines.push_back("Bodies:           " + std::to_string(bodyCount));
        lines.push_back("Broadphase pairs: " + std::to_string(broadphasePairs));
        lines.push_back("Contacts:         " + std::to_string(contactCount));
        for (auto& l : bodyLines) lines.push_back(l);

        // render background quad then text
        static char buf[99999];
        std::vector<float> verts;

        float y = 8.0f;
        for (auto& line : lines) {
            int quads = stb_easy_font_print(8, y, (char*)line.c_str(),
                                            nullptr, buf, sizeof(buf));
            // each quad = 4 verts * 16 bytes
            float* data = (float*)buf;
            for (int q = 0; q < quads * 4; q++) {
                // push all 16 bytes
                verts.insert(verts.end(), data + q*4, data + q*4 + 4);
            }
            y += 12.0f;
        }

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glUseProgram(m_shader);
        GLint loc = glGetUniformLocation(m_shader, "uScreenSize");
        glUniform2f(loc, (float)screenW, (float)screenH);

        glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     verts.size() * sizeof(float),
                     verts.data(), GL_DYNAMIC_DRAW);

        glBindVertexArray(m_vao);
        // stb_easy_font outputs quads but we want triangles, so we need to build an index buffer
        int numQuads = (int)(verts.size() / 16);
        // upload index buffer for quads -> triangles
        std::vector<uint32_t> idx;
        idx.reserve(numQuads * 6);
        for (int q = 0; q < numQuads; q++) {
            uint32_t b = q * 4;
            idx.insert(idx.end(), {b,b+1,b+2, b,b+2,b+3});
        }

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     idx.size()*sizeof(uint32_t),
                     idx.data(), GL_DYNAMIC_DRAW);

        glDrawElements(GL_TRIANGLES, (GLsizei)idx.size(),
                       GL_UNSIGNED_INT, nullptr);

        glBindVertexArray(0);

        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);
    }


private:
    GLuint m_shader, m_vao, m_vbo, m_ebo;

    static std::string fmt(float v, int decimals) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", decimals, v);
        return buf;
    }

    static GLuint compileShader(const char* vert, const char* frag) {
        auto compile = [](GLenum type, const char* src) {
            GLuint id = glCreateShader(type);
            glShaderSource(id, 1, &src, nullptr);
            glCompileShader(id);
            return id;
        };
        GLuint v = compile(GL_VERTEX_SHADER,   vert);
        GLuint f = compile(GL_FRAGMENT_SHADER, frag);
        GLuint p = glCreateProgram();
        glAttachShader(p, v); glAttachShader(p, f);
        glLinkProgram(p);
        glDeleteShader(v); glDeleteShader(f);
        return p;
    }
};


