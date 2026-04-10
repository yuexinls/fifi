#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>
#include <stdexcept>

class Window {
public:
    Window(int width, int height, const std::string& title) {
        if (!glfwInit()) {
            throw std::runtime_error("Failed to init GLFW");
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_SAMPLES, 4);

        m_handle = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
        if (!m_handle)
            throw std::runtime_error("Failed to create window");

        glfwMakeContextCurrent(m_handle);
        glfwSwapInterval(0);

        if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
            throw std::runtime_error("Failed to initialize GLAD");
        
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_MULTISAMPLE);
    }

    ~Window() {
        glfwDestroyWindow(m_handle);
        glfwTerminate();
    }

    bool shouldClose() const { return glfwWindowShouldClose(m_handle); }
    void swapBuffers()       { glfwSwapBuffers(m_handle); }
    void pollEvents()        { glfwPollEvents(); }
    GLFWwindow* handle()     { return m_handle; }

private:
    GLFWwindow* m_handle = nullptr;
};


