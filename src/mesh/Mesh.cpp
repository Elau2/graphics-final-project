#include "mesh/Mesh.h"

#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <limits>
#include <cmath>
#include <numeric>

namespace destruct {

namespace {

// Returns the index of the newly created node in `nodes`.
// Pre-condition: nodes.capacity() >= nodes.size() + 2*(end-start)-1
// (caller does nodes.reserve(2*N) before the first call).
static int buildBVHNode(std::vector<BVHNode>& nodes,
                        const std::vector<glm::vec3>& verts,
                        const std::vector<Tri>& tris,
                        std::vector<int>& idx, int start, int end)
{
    int nodeIdx = (int)nodes.size();
    nodes.push_back({});  // reserve slot (no realloc because of reserve())

    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(-std::numeric_limits<float>::max());
    for (int i = start; i < end; ++i) {
        const Tri& t = tris[idx[i]];
        mn = glm::min(mn, glm::min(verts[t.a], glm::min(verts[t.b], verts[t.c])));
        mx = glm::max(mx, glm::max(verts[t.a], glm::max(verts[t.b], verts[t.c])));
    }
    nodes[nodeIdx].aabbMin = mn;
    nodes[nodeIdx].aabbMax = mx;

    if (end - start == 1) {
        nodes[nodeIdx].triIdx = idx[start];
        return nodeIdx;
    }

    // Split on the longest axis at the midpoint of the node's AABB.
    glm::vec3 ext = mx - mn;
    int axis = (ext.y > ext.x) ? 1 : 0;
    if (ext.z > ext[axis]) axis = 2;
    float mid = 0.5f * (mn[axis] + mx[axis]);

    auto pivot = std::partition(idx.begin() + start, idx.begin() + end,
        [&](int i) {
            const Tri& t = tris[i];
            return (verts[t.a][axis] + verts[t.b][axis] + verts[t.c][axis]) < 3.0f * mid;
        });
    int m = (int)(pivot - idx.begin());
    // Guard against degenerate splits (all centroids on one side).
    if (m == start || m == end) m = (start + end) / 2;

    int left  = buildBVHNode(nodes, verts, tris, idx, start, m);
    int right = buildBVHNode(nodes, verts, tris, idx, m, end);
    // Use index, not pointer/reference — nodes[] is valid because we reserved.
    nodes[nodeIdx].left  = left;
    nodes[nodeIdx].right = right;
    return nodeIdx;
}

} // anonymous namespace

void Mesh::buildBVH()
{
    bvh.clear();
    if (triangles.empty() || vertices.empty()) return;
    std::vector<int> idx(triangles.size());
    std::iota(idx.begin(), idx.end(), 0);
    // A full binary tree with N leaves has at most 2N-1 nodes.
    bvh.reserve(2 * triangles.size());
    buildBVHNode(bvh, vertices, triangles, idx, 0, (int)triangles.size());
}

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
