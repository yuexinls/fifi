#include "core/Window.h"
#include "renderer/Shader.h"
#include "renderer/Mesh.h"
#include "renderer/Camera.h"
#include "physics/PhysicsWorld.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>

// camera and input state
Camera camera;

void mouseMoveCB(GLFWwindow*, double x, double y) {
    camera.onMouseMove(x, y);
}

int main() {
    try {
        Window window(1280, 720, "fifi Engine");
        glfwSetCursorPosCallback  (window.handle(), mouseMoveCB);
        glfwSetInputMode(window.handle(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        Shader shader("shaders/basic.vert", "shaders/basic.frag");
        Mesh   cubeMesh   = Mesh::createCube();
        Mesh   sphereMesh = Mesh::createSphere();

        // build scene
        PhysicsWorld world;
        float floorHalfHeight = 0.5f;
        world.groundY = -3.0f;

        auto* floor = world.addBody(RigidBody::createStatic(
            {0, world.groundY - floorHalfHeight, 0},
            {6, floorHalfHeight, 6}));

        // a stack of boxes
        auto* b0 = world.addBody(RigidBody::createBox(1.0f, {0.5f,0.5f,0.5f}, {0, 4,   0}));
        auto* b1 = world.addBody(RigidBody::createBox(1.0f, {0.5f,0.5f,0.5f}, {1, 5.5f, 0}));
        auto* b2 = world.addBody(RigidBody::createBox(2.0f, {0.7f,0.7f,0.7f}, {2, 7, 0}));

        // a little bit of spin to see angular integration
        b0->angularVelocity = { 0.5f, 1.2f, 0.3f };
        b1->angularVelocity = {-0.8f, 0.4f, 1.0f };

        // a sphere
        auto* sphere = world.addBody(RigidBody::createSphere(1.5f, 0.6f, {-1, 5, 0}));
        sphere->color = { 1.0f, 0.5f, 0.3f };
        sphere->linearVelocity = { -1.0f, 0, 0 };

        // fixed timestep
        const double FIXED_DT = 1.0 / 120.0; // 120 hz
        double accumulator = 0.0;
        double lastTime = glfwGetTime();

        while (!window.shouldClose()) {
            window.pollEvents();

            if (glfwGetKey(window.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window.handle(), true);

            double now   = glfwGetTime();
            double dt    = std::min(now - lastTime, 0.1);
            lastTime     = now;
            accumulator += dt;

            while (accumulator >= FIXED_DT) {
                world.step((float)FIXED_DT);
                accumulator -= FIXED_DT;
            }

            camera.update(window.handle(), (float)dt);

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

                switch (body->shape) {
                    case ShapeType::Sphere: sphereMesh.draw(); break;
                    case ShapeType::Box:    cubeMesh.draw();   break;
                }
            }

            window.swapBuffers();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cin.get();
        return -1;
    }
}


