#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <cmath>

class Camera {
public:
    glm::vec3 position    = { 0.0f, 2.0f, 10.0f };
    float yaw             = -90.0f;
    float pitch           =   0.0f;
    float fov             =  60.0f;
    float moveSpeed       =   5.0f;
    float lookSensitivity = 0.4f;

    void update(GLFWwindow* window, float dt) {
        glm::vec3 fwd   = forward();
        glm::vec3 right = glm::normalize(glm::cross(fwd, worldUp));

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) position += fwd   * moveSpeed * dt;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) position -= fwd   * moveSpeed * dt;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) position -= right * moveSpeed * dt;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) position += right * moveSpeed * dt;

        // Vertical (world up/down)
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) position += worldUp * moveSpeed * dt;
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) position -= worldUp * moveSpeed * dt;

        // Speed modifier
        if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
            moveSpeed = 15.0f;
        else
            moveSpeed = 5.0f;
    }

    void onMouseMove(double x, double y) {
        if (m_firstMouse) {
            m_lastX = x; m_lastY = y;
            m_firstMouse = false;
        }

        float dx = (float)(x - m_lastX) *  lookSensitivity;
        float dy = (float)(y - m_lastY) * -lookSensitivity; // invert Y
        m_lastX = x; m_lastY = y;

        yaw   += dx;
        pitch  = std::clamp(pitch + dy, -89.0f, 89.0f);
    }

    glm::mat4 view() const {
        return glm::lookAt(position, position + forward(), worldUp);
    }

    glm::mat4 projection(float aspect) const {
        return glm::perspective(glm::radians(fov), aspect, 0.1f, 500.0f);
    }

    glm::vec3 eye() const { return position; }

private:
    glm::vec3 worldUp    = { 0, 1, 0 };
    bool      m_firstMouse = true;
    double    m_lastX = 0, m_lastY = 0;

    glm::vec3 forward() const {
        return glm::normalize(glm::vec3(
            std::cos(glm::radians(yaw)) * std::cos(glm::radians(pitch)),
            std::sin(glm::radians(pitch)),
            std::sin(glm::radians(yaw)) * std::cos(glm::radians(pitch))
        ));
    }
};


