#pragma once
// Plane.h
// A plane in 3D, represented as { x : dot(normal, x) = d }.
// By convention, the "inside" half-space is { x : dot(normal, x) <= d }.
// This file is header-only because Plane is a tiny value type.

#include <glm/glm.hpp>

namespace destruct {

struct Plane {
    glm::vec3 normal{0.0f, 1.0f, 0.0f}; // unit length
    float     d = 0.0f;                 // plane equation: dot(normal, x) = d

    Plane() = default;
    Plane(const glm::vec3& n, float d_) : normal(glm::normalize(n)), d(d_) {}

    // Plane that passes through `point` with unit normal `n`.
    static Plane fromPointNormal(const glm::vec3& point, const glm::vec3& n) {
        glm::vec3 nn = glm::normalize(n);
        return Plane(nn, glm::dot(nn, point));
    }

    // Perpendicular bisector between two points. By convention, the resulting
    // plane's normal points from `a` toward `b`, so the "inside" half-space
    // (n . x <= d) contains `a`. This is exactly what we need for Voronoi:
    // to keep the cell of `a`, we clip away the side that contains `b`.
    static Plane bisector(const glm::vec3& a, const glm::vec3& b) {
        glm::vec3 mid  = 0.5f * (a + b);
        glm::vec3 nrm  = glm::normalize(b - a);
        return Plane(nrm, glm::dot(nrm, mid));
    }

    // Signed distance (positive = outside, the side the normal points to).
    float signedDistance(const glm::vec3& p) const {
        return glm::dot(normal, p) - d;
    }
};

} // namespace destruct
