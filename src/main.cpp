#include "core/Window.h"
#include "renderer/Shader.h"
#include "renderer/Mesh.h"
#include "renderer/Camera.h"
#include "physics/PhysicsWorld.h"
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
        Mesh   cubeMesh = Mesh::createCube();

        // build scene
        PhysicsWorld world;
        world.groundY = -3.0f;

        // static floor
        auto* floor = world.addBody(RigidBody::createStatic(
            {0, -4.0f, 0}, {6, 0.5f, 6}));

        // a stack of boxes
        auto* b0 = world.addBody(RigidBody::createBox(1.0f, {0.5f,0.5f,0.5f}, {0, 4,   0}));
        auto* b1 = world.addBody(RigidBody::createBox(1.0f, {0.5f,0.5f,0.5f}, {0, 5.5f, 0}));
        auto* b2 = world.addBody(RigidBody::createBox(2.0f, {0.7f,0.7f,0.7f}, {0.3f, 7, 0}));

        // a little bit of spin to see angular integration
        b0->angularVelocity = { 0.5f, 1.2f, 0.3f };
        b1->angularVelocity = {-0.8f, 0.4f, 1.0f };

        // a sphere
        auto* sphere = world.addBody(RigidBody::createSphere(1.5f, 0.6f, {2, 5, 0}));
        sphere->color = { 1.0f, 0.5f, 0.3f };
        sphere->linearVelocity = { -1.0f, 0, 0 };

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
                world.step((float)FIXED_DT);
                accumulator -= FIXED_DT;
            }

            int w, h;
            glfwGetFramebufferSize(window.handle(), &w, &h);
            glViewport(0, 0, w, h);
            glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            float aspect = (float)w / (float)h;

            shader.bind();
            shader.setMat4 ("uView",       camera.view());
            shader.setMat4 ("uProjection", camera.projection(aspect));
            shader.setVec3 ("uLightPos",   {5, 10, 5});
            shader.setVec3 ("uViewPos",    camera.eye());

            // Draw every body
            for (auto& body : world.bodies) {
                Mat4 model = body->modelMatrix();
                shader.setMat4("uModel", model.toGlm());
                shader.setVec3("uColor", body->color.toGlm());
                cubeMesh.draw(); // all shapes use cube mesh for now
            }

            window.swapBuffers();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cin.get();
        return -1;
    }
}


