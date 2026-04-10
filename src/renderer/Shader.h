#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <stdexcept>
#include <fstream>
#include <iterator>

class Shader {
public:
    Shader(const std::string& vertPath, const std::string& fragPath) {
        std::string vertSrc = readFile(vertPath);
        std::string fragSrc = readFile(fragPath);

        GLuint vert = compile(GL_VERTEX_SHADER,     vertSrc);
        GLuint frag = compile(GL_FRAGMENT_SHADER,   fragSrc);

        m_id = glCreateProgram();
        glAttachShader(m_id, vert);
        glAttachShader(m_id, frag);
        glLinkProgram(m_id);
        checkLink(m_id);

        glDeleteShader(vert);
        glDeleteShader(frag);
    }

    ~Shader() { glDeleteProgram(m_id); }

    void bind()     const { glUseProgram(m_id); }
    void unbind()   const { glUseProgram(0); }

    // uniform setters
    void setMat4 (const std::string& name, const glm::mat4& v) const {
        glUniformMatrix4fv(loc(name), 1, GL_FALSE, &v[0][0]);
    }
    void setVec3 (const std::string& name, const glm::vec3& v) const {
        glUniform3fv(loc(name), 1, &v[0]);
    }
    void setFloat(const std::string& name, float v) const {
        glUniform1f(loc(name), v);
    }
    void setInt  (const std::string& name, int v) const {
        glUniform1i(loc(name), v);
    }

private:
    GLuint m_id;

    GLint loc(const std::string& name) const {
        return glGetUniformLocation(m_id, name.c_str());
    }

    static std::string readFile(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open())
            throw std::runtime_error("Failed to open shader: " + path);
        return { std::istreambuf_iterator<char>(file),
                 std::istreambuf_iterator<char>() };
    }

    static GLuint compile(GLenum type, const std::string& src) {
        GLuint id = glCreateShader(type);
        const char* c = src.c_str();
        glShaderSource(id, 1, &c, nullptr);
        glCompileShader(id);

        GLint ok; glGetShaderiv(id, GL_COMPILE_STATUS, &ok);
        if (!ok) {
            char log[512]; glGetShaderInfoLog(id, 512, nullptr, log);
            throw std::runtime_error(std::string("Shader compile error:\n") + log);
        }
        return id;
    }

    static void checkLink(GLuint id) {
        GLint ok; glGetProgramiv(id, GL_LINK_STATUS, &ok);
        if (!ok) {
            char log[512]; glGetProgramInfoLog(id, 512, nullptr, log);
            throw std::runtime_error(std::string("Shader link error:\n") + log);
        }
    }
};


