#include "core/Window.h"
#include "renderer/Shader.h"
#include "renderer/Mesh.h"
#include "renderer/Camera.h"
#include "renderer/LineRenderer.h"
#include "renderer/DebugOverlay.h"
#include "physics/PhysicsWorld.h"
#include "physics/PhysicsWatchdog.h"
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
        g_camera = &camera;

        DebugOverlay debugOverlay;
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

        // floor
        auto* floor = world.addBody(RigidBody::createStatic(
            {0, world.groundY - floorHalfHeight, 0},
            {64, floorHalfHeight, 64}));

        // ramp
        auto* ramp = world.addBody(RigidBody::createStatic({3, -1.5f, 0}, {3, 0.2f, 2}));
        ramp->orientation           = Quaternion::fromAxisAngle({0,0,1}, 0.35f);
        ramp->collider.halfExtents  = {3, 0.2f, 2};

        // stack of four boxes
        for (int i = 0; i < 4; i++) {
            auto* b = world.addBody(
                RigidBody::createBox(1.0f, {0.5f, 0.5f, 0.5f},
                                    {-3.0f, -2.5f + i * 1.2f, 0.0f}));
            b->restitution = 0.1f;
            b->friction    = 0.7f;
        }

        // heavy ball
        auto* ball = world.addBody(RigidBody::createSphere(3.0f, 0.6f, {6.0f, 1.0f, 0}));
        ball->color          = {1.0f, 0.4f, 0.2f};
        ball->restitution    = 0.3f;
        ball->friction       = 0.5f;
        ball->linearVelocity = {-4.0f, 0, 0};

        // bouncy ball 🤑
        auto* bouncy = world.addBody(RigidBody::createSphere(0.5f, 0.4f, {0.0f, 4.0f, 0}));
        bouncy->color       = {0.3f, 1.0f, 0.4f};
        bouncy->restitution = 0.8f;
        bouncy->friction    = 0.2f;

        // i love balls

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

            // draw every body
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
                // green  = clear
                // yellow = broadphase candidate
                std::vector<bool> inBroadphase(world.bodies.size(), false);
                for (auto& [i,j] : world.broadphasePairs) {
                    inBroadphase[i] = inBroadphase[j] = true;
                }

                for (int i = 0; i < (int)world.bodies.size(); i++) {
                    auto& b    = world.bodies[i];
                    auto  aabb = b->collider.computeAABB(b->position, b->orientation);

                    glm::vec3 col;
                    if (b->isStatic()) {
                        col = { 0.3f, 0.3f, 1.0f };     // blue   = static body
                    } else if (inBroadphase[i]) {
                        col = { 1.0f, 1.0f, 0.0f };     // yellow = broadphase candidate
                    } else {
                        col = { 0.0f, 1.0f, 0.0f };     // green  = clear
                    }

                    lines.addAABB(
                        {aabb.min.x, aabb.min.y, aabb.min.z},
                        {aabb.max.x, aabb.max.y, aabb.max.z}, col);
                }

                // contact points and normals (red = normal, orange = tangents)
                for (auto& c : world.contacts) {
                    glm::vec3 cp = { c.contactPoint.x, c.contactPoint.y, c.contactPoint.z };
                    glm::vec3 cn = { c.normal.x,       c.normal.y,       c.normal.z       };

                    // normal line scaled by penetration depth (longer = deeper penetration)
                    float scale = 1.0f + c.penetrationDepth * 5.0f;
                    lines.addLine(cp, cp + cn * scale, {1, 0, 0});     // red = normal

                    // tangent cross marker at contact point
                    glm::vec3 perp = glm::abs(cn.y) < 0.9f
                                ? glm::vec3{0,1,0} : glm::vec3{1,0,0};
                    glm::vec3 t1 = glm::normalize(glm::cross(cn, perp)) * 0.15f;
                    glm::vec3 t2 = glm::normalize(glm::cross(cn, t1))   * 0.15f;

                    lines.addLine(cp - t1, cp + t1, {1, 0.5f, 0});     // orange = tangent X
                    lines.addLine(cp - t2, cp + t2, {1, 0.5f, 0});     // orange = tangent Y
                }

                lineShader.bind();
                lineShader.setMat4("uView",       view);
                lineShader.setMat4("uProjection", proj);
                lines.draw();
            }

            // build text lines for each body
            std::vector<std::string> bodyLines;
            auto& wdBodies = world.watchdog.report.bodies;

            for (int i = 0; i < (int)world.bodies.size(); i++) {
                auto& b = world.bodies[i];

                std::ostringstream ss;
                ss << std::fixed << std::setprecision(2);
                ss << "[" << i << "] "
                << "pos(" << b->position.x << "," << b->position.y << "," << b->position.z << ") ";

                // guard
                if (i < (int)wdBodies.size()) {
                    auto& wd = wdBodies[i];
                    ss << "spd=" << wd.linearSpeed << " "
                    << "wSpd=" << wd.angularSpeed;
                    if (wd.speedWarning) ss << " [fast]";
                    if (wd.nanWarning)   ss << " [NaN]";
                } else {
                    ss << "spd=-- wSpd=--";
                }

                if (b->isStatic()) ss << " [static]";
                bodyLines.push_back(ss.str());
            }

            // draw hud
            debugOverlay.draw(w, h, fps,
                              (int)world.bodies.size(),
                              (int)world.broadphasePairs.size(),
                              (int)world.contacts.size(),
                              world.watchdog.report,
                              bodyLines);

            window.swapBuffers();
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cin.get();
        return -1;
    }
}


