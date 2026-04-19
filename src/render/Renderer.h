#pragma once
// Renderer.h
//
// Forward renderer that draws the ground plane and one GPUMesh per rigid
// body, each with a per-body colour. Lighting is a simple single-directional-
// light Phong model.
//
// Renderer does not own the meshes; it takes a parallel pair of arrays
// (bodies + their GPU meshes) and draws each in sequence. The app glues
// them together.

#include "render/Shader.h"
#include "render/Camera.h"
#include "render/GPUMesh.h"
#include "physics/RigidBody.h"

#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <string>

namespace destruct {

class Renderer {
public:
    // Background colour.
    glm::vec3 clearColor{0.07f, 0.08f, 0.10f};

    // Directional light (world space, toward the sun).
    glm::vec3 lightDir{-0.4f, -1.0f, -0.3f};
    glm::vec3 lightColor{1.0f, 0.98f, 0.92f};
    glm::vec3 ambient{0.14f, 0.15f, 0.18f};

    Renderer() = default;

    // Load the two shader programs from `shaderDir`. Returns false on
    // compile/link errors (which are also printed to stderr).
    bool init(const std::string& shaderDir);

    // Set the GL viewport and propagate to the camera.
    void setViewport(Camera& cam, int widthPx, int heightPx);

    // Render a frame. `bodies` and `meshes` must be in 1-to-1 correspondence.
    // `colors` is optional; if empty, a stable hash-based palette is used.
    void render(const Camera& cam,
                const std::vector<std::unique_ptr<RigidBody>>& bodies,
                const std::vector<std::unique_ptr<GPUMesh>>&   meshes,
                const std::vector<glm::vec3>&                  colors);

private:
    Shader meshShader_;
    Shader groundShader_;

    uint32_t groundVAO_ = 0;
    uint32_t groundVBO_ = 0;

    void initGroundMesh();
    void drawGround(const Camera& cam);
};

} // namespace destruct
