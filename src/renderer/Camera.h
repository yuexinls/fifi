#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>

class Camera {
public:
    float yaw      = -90.0f;
    float pitch    =  20.0f;
    float distance =   8.0f;
    float fov      =  60.0f;

    glm::mat4 view() const {
        return glm::lookAt(eye(), glm::vec3(0.0f), glm::vec3(0,1,0));
    }

    glm::mat4 projection(float aspect) const {
        return glm::perspective(glm::radians(fov), aspect, 0.1f, 500.0f);
    }

    glm::vec3 eye() const {
        float yR = glm::radians(yaw);
        float pR = glm::radians(pitch);
        return glm::vec3{
            distance * cos(pR) * cos(yR),
            distance * sin(pR),
            distance * cos(pR) * sin(yR)
        };
    }

    // called from mouse drag callback
    void orbit(float dx, float dy) {
        yaw   += dx * 0.4f;
        pitch += dy * 0.4f;
        pitch  = std::clamp(pitch, -89.0f, 89.0f);
    }

    // called from mouse scroll callback
    void zoom(float delta) {
        distance -= delta * 0.5f;
        distance  = std::clamp(distance, 1.0f, 100.0f);
    }
};


