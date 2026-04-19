// Camera.cpp
#include "render/Camera.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include <algorithm>
#include <cmath>

namespace destruct {

glm::vec3 Camera::eye() const
{
    float cp = std::cos(pitch), sp = std::sin(pitch);
    float cy = std::cos(yaw),   sy = std::sin(yaw);
    glm::vec3 dir(cp * sy, sp, cp * cy);
    return target + distance * dir;
}

glm::mat4 Camera::viewMatrix() const
{
    return glm::lookAt(eye(), target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 Camera::projectionMatrix() const
{
    float a = (viewportHeight > 0)
                ? static_cast<float>(viewportWidth) / viewportHeight
                : aspect;
    return glm::perspective(fovY, a, zNear, zFar);
}

void Camera::orbit(float dx, float dy)
{
    yaw   -= dx;
    pitch += dy;
    const float lim = glm::radians(89.0f);
    pitch = std::clamp(pitch, -lim, lim);
}

void Camera::pan(float dx, float dy)
{
    // Translate target in the camera's right/up basis so the view follows
    // the cursor naturally when panning.
    glm::mat4 V = viewMatrix();
    glm::vec3 right(V[0][0], V[1][0], V[2][0]);
    glm::vec3 up   (V[0][1], V[1][1], V[2][1]);
    const float scale = distance * 0.001f;
    target += -dx * scale * right + dy * scale * up;
}

void Camera::dolly(float amount)
{
    // Exponential zoom feels much nicer than linear.
    distance *= std::pow(0.9f, amount);
    distance  = std::clamp(distance, 0.1f, 500.0f);
}

void Camera::getRayFromScreen(double px, double py,
                              glm::vec3& outOrigin,
                              glm::vec3& outDir) const
{
    // NDC ([-1,1], y flipped).
    float x = 2.0f * static_cast<float>(px) / viewportWidth  - 1.0f;
    float y = 1.0f - 2.0f * static_cast<float>(py) / viewportHeight;

    glm::mat4 invVP = glm::inverse(projectionMatrix() * viewMatrix());
    glm::vec4 nearH = invVP * glm::vec4(x, y, -1.0f, 1.0f);
    glm::vec4 farH  = invVP * glm::vec4(x, y,  1.0f, 1.0f);
    glm::vec3 nearP = glm::vec3(nearH) / nearH.w;
    glm::vec3 farP  = glm::vec3(farH)  / farH.w;

    outOrigin = nearP;
    outDir    = glm::normalize(farP - nearP);
}

} // namespace destruct
