#include <core/Window.h>
#include <renderer/Shader.h>
#include <renderer/Mesh.h>
#include <renderer/Camera.h>
#include <math/Mat4.h>
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
        Vec3 a(1,0,0), b(0,1,0);

        // cross product
        assert(a.cross(b) == Vec3(0,0,1));

        // dot product
        assert(a.dot(b) == 0.0f);

        // normalization
        Vec3 v(3,0,0);
        assert(v.normalized() == Vec3(1,0,0));

        // quaternion rotation
        Quaternion q = Quaternion::fromAxisAngle(Vec3(0,0,1), 3.14159f * 0.5f);
        Vec3 rotated = q.rotate(Vec3(1,0,0));
        assert(std::abs(rotated.x) < 1e-5f);
        assert(std::abs(rotated.y - 1.0f) < 1e-5f);

        // Mat4 trs
        Vec3 pos(1,2,3);
        Mat4 t = Mat4::translation(pos);
        Vec3 result = t.transformPoint(Vec3(0,0,0));
        assert(result == pos);

        std::cout << "All math checks passed!\n";

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


