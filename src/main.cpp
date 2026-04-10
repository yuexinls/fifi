#include <core/Window.h>
#include <iostream>

int main() {
    Window window(1280, 720, "fifi Engine");

    // fixed timestep
    const double FIXED_DT = 1.0 / 120.0; // 120 hz
    double accumulator = 0.0;
    double lastTime = glfwGetTime();

    while (!window.shouldClose()) {
        window.pollEvents();

        double now = glfwGetTime();
        double frameTime = now - lastTime;
        lastTime = now;

        // cap frame time to avoid spiral of death
        if (frameTime > 0.25) frameTime = 0.25;
        accumulator += frameTime;

        while (accumulator >= FIXED_DT) {
            // physicsWorld.step(FIXED_DT);
            accumulator -= FIXED_DT;
        }

        // interpolation alpha for rendering between physics steps
        double alpha = accumulator / FIXED_DT;

        glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // renderer.draw(alpha);

        window.swapBuffers();
    }
}