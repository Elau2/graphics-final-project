#pragma once
// Camera.h
//
// An orbit ("arcball") camera around a target point. Mouse drag rotates;
// mouse wheel zooms. getRayFromScreen converts a pixel coordinate to a
// world-space ray for click-to-fracture raycasts (aspirational feature).
//
// Rotation is stored as yaw + pitch (Euler) which is simple and avoids
// gimbal problems in this constrained use-case (pitch is clamped).

#include <glm/glm.hpp>

namespace destruct {

class Camera {
public:
    // Target the camera orbits around. This is the point the view looks at.
    glm::vec3 target{0.0f};

    // Spherical coordinates around target.
    float distance = 4.0f;  // metres
    float yaw      = 0.6f;  // radians, around +Y
    float pitch    = 0.3f;  // radians, above the horizon

    // Projection.
    float fovY     = glm::radians(60.0f);
    float aspect   = 16.0f / 9.0f;
    float zNear    = 0.05f;
    float zFar     = 100.0f;

    // Viewport pixel size. Used by getRayFromScreen.
    int   viewportWidth  = 1280;
    int   viewportHeight = 720;

    Camera() = default;

    // Build matrices from current state.
    glm::mat4 viewMatrix()       const;
    glm::mat4 projectionMatrix() const;
    glm::vec3 eye()              const;

    // Interaction.
    void orbit(float dxRadians, float dyRadians);
    void pan  (float dx, float dy);   // in screen-space-ish world units
    void dolly(float amount);         // positive = zoom in

    // Pixel (x,y) in screen coords with y-down -> world-space ray origin
    // and direction. The direction is unit length.
    void getRayFromScreen(double px, double py,
                          glm::vec3& outOrigin,
                          glm::vec3& outDir) const;
};

} // namespace destruct
