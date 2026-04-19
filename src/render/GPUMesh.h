#pragma once
// GPUMesh.h
//
// A VAO + VBO + EBO trio that owns a copy of a Mesh on the GPU. One
// fragment uses one GPUMesh. `uploadFromMesh` (re)allocates the buffers
// when called, so fragments created mid-simulation just instantiate a new
// GPUMesh.
//
// We interleave position + normal in a single VBO for cache friendliness
// and to keep the vertex spec declaration short.

#include "mesh/Mesh.h"
#include <cstdint>

namespace destruct {

class GPUMesh {
public:
    GPUMesh() = default;
    ~GPUMesh();
    GPUMesh(const GPUMesh&)            = delete;
    GPUMesh& operator=(const GPUMesh&) = delete;
    GPUMesh(GPUMesh&& o) noexcept;
    GPUMesh& operator=(GPUMesh&& o) noexcept;

    // (Re)upload. Normals are required; Mesh::recomputeNormals fills them.
    void uploadFromMesh(const Mesh& mesh);

    // Issue the draw call (assumes the shader and uniforms are already set).
    void draw() const;

    std::size_t indexCount() const { return indexCount_; }
    bool valid() const             { return vao_ != 0 && indexCount_ > 0; }

private:
    uint32_t vao_ = 0;
    uint32_t vbo_ = 0;
    uint32_t ebo_ = 0;
    std::size_t indexCount_ = 0;

    void release();
};

} // namespace destruct
