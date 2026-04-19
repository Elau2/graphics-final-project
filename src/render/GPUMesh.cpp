// GPUMesh.cpp
#include "render/GPUMesh.h"

#include <GL/glew.h>
#include <vector>
#include <utility>

namespace destruct {

GPUMesh::~GPUMesh() { release(); }

GPUMesh::GPUMesh(GPUMesh&& o) noexcept
    : vao_(o.vao_), vbo_(o.vbo_), ebo_(o.ebo_), indexCount_(o.indexCount_)
{
    o.vao_ = o.vbo_ = o.ebo_ = 0;
    o.indexCount_ = 0;
}

GPUMesh& GPUMesh::operator=(GPUMesh&& o) noexcept
{
    if (this != &o) {
        release();
        vao_ = o.vao_; vbo_ = o.vbo_; ebo_ = o.ebo_;
        indexCount_ = o.indexCount_;
        o.vao_ = o.vbo_ = o.ebo_ = 0;
        o.indexCount_ = 0;
    }
    return *this;
}

void GPUMesh::release()
{
    if (ebo_) { glDeleteBuffers(1, &ebo_); ebo_ = 0; }
    if (vbo_) { glDeleteBuffers(1, &vbo_); vbo_ = 0; }
    if (vao_) { glDeleteVertexArrays(1, &vao_); vao_ = 0; }
    indexCount_ = 0;
}

void GPUMesh::uploadFromMesh(const Mesh& mesh)
{
    if (mesh.empty()) { release(); return; }

    // Interleave pos + normal. If normals are absent, zero-fill (shader
    // will still work but lighting will be flat/wrong).
    const std::size_t nVerts = mesh.vertices.size();
    std::vector<float> interleaved(nVerts * 6);
    const bool haveN = mesh.normals.size() == nVerts;
    for (std::size_t i = 0; i < nVerts; ++i) {
        const auto& p = mesh.vertices[i];
        interleaved[i * 6 + 0] = p.x;
        interleaved[i * 6 + 1] = p.y;
        interleaved[i * 6 + 2] = p.z;
        if (haveN) {
            const auto& n = mesh.normals[i];
            interleaved[i * 6 + 3] = n.x;
            interleaved[i * 6 + 4] = n.y;
            interleaved[i * 6 + 5] = n.z;
        } else {
            interleaved[i * 6 + 3] = 0.0f;
            interleaved[i * 6 + 4] = 1.0f;
            interleaved[i * 6 + 5] = 0.0f;
        }
    }

    std::vector<uint32_t> indices(mesh.triangles.size() * 3);
    for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
        indices[i * 3 + 0] = mesh.triangles[i].a;
        indices[i * 3 + 1] = mesh.triangles[i].b;
        indices[i * 3 + 2] = mesh.triangles[i].c;
    }

    if (!vao_) glGenVertexArrays(1, &vao_);
    if (!vbo_) glGenBuffers(1, &vbo_);
    if (!ebo_) glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(interleaved.size() * sizeof(float)),
                 interleaved.data(),
                 GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)),
                 indices.data(),
                 GL_STATIC_DRAW);

    const GLsizei stride = 6 * sizeof(float);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride,
                          (void*)(3 * sizeof(float)));

    glBindVertexArray(0);

    indexCount_ = indices.size();
}

void GPUMesh::draw() const
{
    if (!valid()) return;
    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(indexCount_),
                   GL_UNSIGNED_INT,
                   (void*)0);
    glBindVertexArray(0);
}

} // namespace destruct
