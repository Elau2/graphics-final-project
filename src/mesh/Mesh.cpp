#include "mesh/Mesh.h"

#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <limits>
#include <cmath>

namespace destruct {

void Mesh::clear() {
    vertices.clear();
    normals.clear();
    triangles.clear();
}

void Mesh::recomputeNormals() {
    normals.assign(vertices.size(), glm::vec3(0.0f));
    // Accumulate area-weighted face normals on each triangle's 3 vertices.
    for (const Tri& t : triangles) {
        const glm::vec3& A = vertices[t.a];
        const glm::vec3& B = vertices[t.b];
        const glm::vec3& C = vertices[t.c];
        glm::vec3 faceN = glm::cross(B - A, C - A);
        // length of the cross product is 2 * area, so this is area-weighted.
        normals[t.a] += faceN;
        normals[t.b] += faceN;
        normals[t.c] += faceN;
    }
    for (glm::vec3& n : normals) {
        float len = glm::length(n);
        if (len > 1e-12f) n /= len;
        else              n = glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

void Mesh::computeAABB(glm::vec3& outMin, glm::vec3& outMax) const {
    if (vertices.empty()) {
        outMin = glm::vec3(0.0f);
        outMax = glm::vec3(0.0f);
        return;
    }
    outMin = glm::vec3(std::numeric_limits<float>::max());
    outMax = glm::vec3(-std::numeric_limits<float>::max());
    for (const glm::vec3& v : vertices) {
        outMin = glm::min(outMin, v);
        outMax = glm::max(outMax, v);
    }
}

void Mesh::translate(const glm::vec3& delta) {
    for (glm::vec3& v : vertices) v += delta;
}

void Mesh::append(const Mesh& other) {
    const uint32_t base = static_cast<uint32_t>(vertices.size());
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    if (!other.normals.empty()) {
        if (normals.size() != base) normals.resize(base, glm::vec3(0.0f));
        normals.insert(normals.end(), other.normals.begin(), other.normals.end());
    }
    triangles.reserve(triangles.size() + other.triangles.size());
    for (const Tri& t : other.triangles) {
        triangles.emplace_back(t.a + base, t.b + base, t.c + base);
    }
}

Mesh Mesh::makeCube(float h) {
    Mesh m;
    m.vertices = {
        {-h, -h, -h}, { h, -h, -h}, { h,  h, -h}, {-h,  h, -h},
        {-h, -h,  h}, { h, -h,  h}, { h,  h,  h}, {-h,  h,  h},
    };
    // 12 triangles, two per face, CCW when viewed from outside.
    m.triangles = {
        {0, 3, 2}, {0, 2, 1}, // -Z
        {4, 5, 6}, {4, 6, 7}, // +Z
        {0, 4, 7}, {0, 7, 3}, // -X
        {1, 2, 6}, {1, 6, 5}, // +X
        {0, 1, 5}, {0, 5, 4}, // -Y
        {3, 7, 6}, {3, 6, 2}, // +Y
    };
    m.recomputeNormals();
    return m;
}

Mesh Mesh::makeSphere(float radius, int slices, int stacks) {
    Mesh m;
    slices = std::max(slices, 3);
    stacks = std::max(stacks, 2);

    // Vertices along parallels (top and bottom are collapsed below).
    for (int i = 0; i <= stacks; ++i) {
        float v     = static_cast<float>(i) / stacks;
        float phi   = v * glm::pi<float>();          // 0 .. pi
        float sphi  = std::sin(phi), cphi = std::cos(phi);
        for (int j = 0; j <= slices; ++j) {
            float u     = static_cast<float>(j) / slices;
            float theta = u * glm::two_pi<float>();
            float stheta = std::sin(theta), ctheta = std::cos(theta);
            m.vertices.emplace_back(radius * sphi * ctheta,
                                    radius * cphi,
                                    radius * sphi * stheta);
        }
    }

    int ring = slices + 1;
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            uint32_t a = i * ring + j;
            uint32_t b = a + 1;
            uint32_t c = a + ring;
            uint32_t d = c + 1;
            m.triangles.emplace_back(a, c, d);
            m.triangles.emplace_back(a, d, b);
        }
    }
    m.recomputeNormals();
    return m;
}

} // namespace destruct
