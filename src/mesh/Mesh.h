#pragma once
// Mesh.h
// A plain triangle-soup mesh. We deliberately avoid a half-edge structure for
// fragments: after Voronoi clipping the resulting fragments are small and
// their topology is rebuilt from scratch each time, so the overhead of a
// half-edge data structure is not worth it. The core algorithms (clipping,
// mass properties, GPU upload) all operate directly on vertex + triangle
// lists.

#include <glm/glm.hpp>
#include <vector>
#include <string>

namespace destruct {

// A triangle is three indices into the vertex array.
struct Tri {
    uint32_t a = 0, b = 0, c = 0;
    Tri() = default;
    Tri(uint32_t a_, uint32_t b_, uint32_t c_) : a(a_), b(b_), c(c_) {}
};

class Mesh {
public:
    // Raw geometry. We keep them public because the mesh pipeline (clipping,
    // Voronoi) rebuilds meshes frequently and a getter/setter layer would
    // just add noise.
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;    // per-vertex normals (may be empty until computed)
    std::vector<Tri>       triangles;

    Mesh() = default;

    // Basic info.
    std::size_t vertexCount() const   { return vertices.size(); }
    std::size_t triangleCount() const { return triangles.size(); }
    bool empty() const                { return triangles.empty() || vertices.empty(); }

    // Clear all geometry.
    void clear();

    // Build per-vertex normals by averaging the face normals of adjacent
    // triangles (weighted by face area). Safe to call on a freshly clipped
    // mesh.
    void recomputeNormals();

    // Axis-aligned bounding box.
    void computeAABB(glm::vec3& outMin, glm::vec3& outMax) const;

    // Translate every vertex by `delta`. Normals are unchanged.
    void translate(const glm::vec3& delta);

    // Merge another mesh into this one, appending its vertices and triangles.
    // Indices are remapped correctly.
    void append(const Mesh& other);

    // Factory: a unit-ish cube centred at the origin, 12 triangles, 8 vertices.
    // Handy when no OBJ is supplied and for smoke tests of the clipper.
    static Mesh makeCube(float halfSize = 0.5f);

    // Factory: a UV sphere. Useful for the "shatter a sphere" demo.
    static Mesh makeSphere(float radius, int slices, int stacks);
};

} // namespace destruct
