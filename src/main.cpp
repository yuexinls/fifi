#include <core/Window.h>
#include <renderer/Shader.h>
#include <renderer/Mesh.h>
#include <renderer/Camera.h>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

// input state
Camera camera;
bool mouseDown = false;
double lastX = 0.0, lastY = 0.0;

void mouseBtnCB(GLFWwindow*, int btn, int action, int) {
    if (btn == GLFW_MOUSE_BUTTON_LEFT)
        mouseDown = (action == GLFW_PRESS);
}

void mouseMoveCB(GLFWwindow*, double x, double y) {
    if (mouseDown) {
        camera.orbit((float)(x - lastX), (float)(lastY - y));
    }
    lastX = x; lastY = y;
}

void scrollCB(GLFWwindow*, double, double dy) {
    camera.zoom((float)dy);
}

int main() {
    try {
        Window window(1280, 720, "fifi Engine");
        glfwSetMouseButtonCallback(window.handle(), mouseBtnCB);
        glfwSetCursorPosCallback  (window.handle(), mouseMoveCB);
        glfwSetScrollCallback     (window.handle(), scrollCB);

        Shader shader("shaders/basic.vert", "shaders/basic.frag");
        Mesh   cube = Mesh::createCube();

        // fixed timestep
        const double FIXED_DT = 1.0 / 120.0; // 120 hz
        double accumulator = 0.0;
        double lastTime = glfwGetTime();

        while (!window.shouldClose()) {
            window.pollEvents();

            double now   = glfwGetTime();
            double dt    = std::min(now - lastTime, 0.25);
            lastTime     = now;
            accumulator += dt;

            while (accumulator >= FIXED_DT) {
                // physicsWorld.step(FIXED_DT); <- later
                accumulator -= FIXED_DT;
            }

            int w, h;
            glfwGetFramebufferSize(window.handle(), &w, &h);
            glViewport(0, 0, w, h);
            glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            float aspect = (float)w / (float)h;

            // rotate cube slowly aww its so cute
            float angle = (float)glfwGetTime() * 30.0f;
            glm::mat4 model = glm::rotate(glm::mat4(1.0f),
                                        glm::radians(angle),
                                        glm::vec3(0.3f, 1.0f, 0.2f));

            shader.bind();
            shader.setMat4 ("uModel",      model);
            shader.setMat4 ("uView",       camera.view());
            shader.setMat4 ("uProjection", camera.projection(aspect));
            shader.setVec3 ("uColor",      {0.4f, 0.6f, 1.0f});
            shader.setVec3 ("uLightPos",   {5.0f, 8.0f, 5.0f});
            shader.setVec3 ("uViewPos",    camera.eye());

            cube.draw();

            window.swapBuffers();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cin.get();
        return -1;
    }
}


