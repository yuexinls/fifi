#include "core/Window.h"
#include "renderer/Shader.h"
#include "renderer/Mesh.h"
#include "renderer/Camera.h"
#include "renderer/LineRenderer.h"
#include "renderer/DebugOverlay.h"
#include "physics/PhysicsWorld.h"
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>

// camera and input state
Camera*       g_camera       = nullptr;
DebugOverlay* g_debugOverlay = nullptr;

void mouseMoveCB(GLFWwindow*, double x, double y) {
    g_camera->onMouseMove(x, y);
}
void keyCB(GLFWwindow*, int key, int, int action, int) {
    g_debugOverlay->onKey(key, action);
}

int main() {
    try {
        Window window(1280, 720, "fifi Engine");

        Camera camera;
        DebugOverlay debugOverlay;
        g_camera = &camera;
        g_debugOverlay = &debugOverlay;

        glfwSetCursorPosCallback(window.handle(), mouseMoveCB);
        glfwSetInputMode        (window.handle(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
        glfwSetKeyCallback      (window.handle(), keyCB);

        Shader shader("shaders/basic.vert",    "shaders/basic.frag");
        Shader lineShader("shaders/line.vert", "shaders/line.frag");
        Mesh   cubeMesh   = Mesh::createCube();
        Mesh   sphereMesh = Mesh::createSphere();
        LineRenderer lines;

        // build scene
        PhysicsWorld world;
        float floorHalfHeight = 0.5f;
        world.groundY = -3.0f;

        auto* floor = world.addBody(RigidBody::createStatic(
            {0, world.groundY - floorHalfHeight, 0},
            {6, floorHalfHeight, 6}));

        // a stack of objects
        auto* b0 = world.addBody(RigidBody::createBox  (1.0f,{0.5f,0.5f,0.5f},{0,  4,  0}));
        auto* b1 = world.addBody(RigidBody::createBox  (1.0f,{0.5f,0.5f,0.5f},{0,  6,  0}));
        auto* sp = world.addBody(RigidBody::createSphere(1.5f, 0.6f,           {2,  5,  0}));
        b0->angularVelocity = { 0.5f, 1.2f, 0.3f };
        sp->color = { 1.0f, 0.5f, 0.3f };

        // fixed timestep
        const double FIXED_DT = 1.0 / 120.0; // 120 hz
        double accumulator = 0.0;
        double lastTime = glfwGetTime();

        // fps smoothing
        float  fps        = 0.0f;
        double fpsTimer   = 0.0;
        int    fpsFrames  = 0;

        while (!window.shouldClose()) {
            window.pollEvents();

            if (glfwGetKey(window.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
                glfwSetWindowShouldClose(window.handle(), true);

            double now   = glfwGetTime();
            double dt    = std::min(now - lastTime, 0.1);
            lastTime     = now;
            accumulator += dt;

            camera.update(window.handle(), (float)dt);

            while (accumulator >= FIXED_DT) {
                world.step((float)FIXED_DT);
                accumulator -= FIXED_DT;
            }

            // fps counter
            fpsFrames++;
            fpsTimer += dt;
            if (fpsTimer >= 0.25) {
                fps      = (float)fpsFrames / (float)fpsTimer;
                fpsTimer = fpsFrames = 0;
            }

            int w, h;
            glfwGetFramebufferSize(window.handle(), &w, &h);
            glViewport(0, 0, w, h);
            glClearColor(0.4f, 0.8f, 1.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            float aspect = (float)w / (float)h;
            auto  view   = camera.view();
            auto  proj   = camera.projection(aspect);

            shader.bind();
            shader.setMat4 ("uView",       view);
            shader.setMat4 ("uProjection", proj);
            shader.setVec3 ("uLightPos",   {5, 10, 5});
            shader.setVec3 ("uViewPos",    camera.eye());

            // Draw every body
            for (auto& body : world.bodies) {
                shader.setMat4("uModel", body->modelMatrix().toGlm());
                shader.setVec3("uColor", body->color.toGlm());
                switch (body->shape) {
                    case ShapeType::Sphere: sphereMesh.draw(); break;
                    case ShapeType::Box:    cubeMesh.draw();   break;
                }
            }

            // debug overlay
            if (debugOverlay.visible) {
                lines.begin();

                // AABB for each body
                // (green = clear, yellow = broadphase candidate)
                std::vector<bool> inBroadphase(world.bodies.size(), false);
                for (auto& [i,j] : world.broadphasePairs) {
                    inBroadphase[i] = inBroadphase[j] = true;
                }

                for (int i = 0; i < (int)world.bodies.size(); i++) {
                    auto& b    = world.bodies[i];
                    auto  aabb = b->collider.computeAABB(b->position, b->orientation);

                    glm::vec3 col;
                    if (b->isStatic()) {
                        col = { 0.3f, 0.3f, 1.0f };        // blue  = static body
                    } else if (inBroadphase[i]) {
                        col = { 1.0f, 1.0f, 0.0f };        // yellow = broadphase candidate
                    } else {
                        col = { 0.0f, 1.0f, 0.0f };        // green  = clear
                    }

                    lines.addAABB(
                        {aabb.min.x, aabb.min.y, aabb.min.z},
                        {aabb.max.x, aabb.max.y, aabb.max.z}, col);
                }

                // contact normals (red line from contact point along normal)
                for (auto& c : world.contacts) {
                    glm::vec3 cp = { c.contactPoint.x,
                                     c.contactPoint.y,
                                     c.contactPoint.z };
                    glm::vec3 cn = { c.normal.x, c.normal.y, c.normal.z };
                    lines.addLine(cp, cp + cn * 0.5f, {1,0,0});
                }

                lineShader.bind();
                lineShader.setMat4("uView",       view);
                lineShader.setMat4("uProjection", proj);
                lines.draw();
            }

            // build text lines for each body
            std::vector<std::string> bodyLines;
            for (int i = 0; i < (int)world.bodies.size(); i++) {
                auto& b = world.bodies[i];
                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2);
                ss << "[" << i << "] "
                   << "pos("  << b->position.x       << ","
                               << b->position.y       << ","
                               << b->position.z       << ") "
                   << "vel("  << b->linearVelocity.x  << ","
                               << b->linearVelocity.y  << ","
                               << b->linearVelocity.z  << ") "
                   << (b->isStatic() ? "[static]" : "");
                bodyLines.push_back(ss.str());
            }

            // draw hud
            debugOverlay.draw(w, h, fps,
                              (int)world.bodies.size(),
                              (int)world.broadphasePairs.size(),
                              (int)world.contacts.size(),
                              bodyLines);

            window.swapBuffers();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cin.get();
        return -1;
    }
}


