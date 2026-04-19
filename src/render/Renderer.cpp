// Renderer.cpp
#include "render/Renderer.h"

#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>

#include <cstdint>
#include <iostream>

namespace destruct {

namespace {

// Simple stable hash -> pleasant HSV->RGB conversion. Used when the caller
// doesn't supply per-body colours, so fragments are visually distinct.
glm::vec3 paletteColor(std::size_t i)
{
    // Golden-ratio hue step spreads hues evenly regardless of count.
    const float phi = 0.61803398875f;
    float h = std::fmod(i * phi + 0.13f, 1.0f);
    float s = 0.55f;
    float v = 0.92f;

    float c = v * s;
    float hp = h * 6.0f;
    float x = c * (1.0f - std::fabs(std::fmod(hp, 2.0f) - 1.0f));
    float r1 = 0, g1 = 0, b1 = 0;
    if      (hp < 1) { r1 = c; g1 = x; }
    else if (hp < 2) { r1 = x; g1 = c; }
    else if (hp < 3) { g1 = c; b1 = x; }
    else if (hp < 4) { g1 = x; b1 = c; }
    else if (hp < 5) { r1 = x; b1 = c; }
    else             { r1 = c; b1 = x; }
    float m = v - c;
    return { r1 + m, g1 + m, b1 + m };
}

} // namespace

bool Renderer::init(const std::string& shaderDir)
{
    if (!meshShader_.loadFromFiles(shaderDir + "/mesh.vert",
                                   shaderDir + "/mesh.frag")) {
        return false;
    }
    if (!groundShader_.loadFromFiles(shaderDir + "/ground.vert",
                                     shaderDir + "/ground.frag")) {
        return false;
    }

    initGroundMesh();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    return true;
}

void Renderer::initGroundMesh()
{
    // A large XZ quad centred at origin.
    const float s = 12.0f;
    float verts[] = {
        // pos.xyz           // normal.xyz
        -s, 0.0f, -s,   0.0f, 1.0f, 0.0f,
         s, 0.0f, -s,   0.0f, 1.0f, 0.0f,
         s, 0.0f,  s,   0.0f, 1.0f, 0.0f,
        -s, 0.0f, -s,   0.0f, 1.0f, 0.0f,
         s, 0.0f,  s,   0.0f, 1.0f, 0.0f,
        -s, 0.0f,  s,   0.0f, 1.0f, 0.0f,
    };

    glGenVertexArrays(1, &groundVAO_);
    glGenBuffers(1, &groundVBO_);
    glBindVertexArray(groundVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, groundVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(verts), verts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glBindVertexArray(0);
}

void Renderer::setViewport(Camera& cam, int widthPx, int heightPx)
{
    glViewport(0, 0, widthPx, heightPx);
    cam.viewportWidth  = widthPx;
    cam.viewportHeight = heightPx;
}

void Renderer::drawGround(const Camera& cam)
{
    if (!groundVAO_) return;
    groundShader_.use();
    groundShader_.setMat4("uView",     cam.viewMatrix());
    groundShader_.setMat4("uProj",     cam.projectionMatrix());
    groundShader_.setVec3("uLightDir", glm::normalize(lightDir));
    groundShader_.setVec3("uLightCol", lightColor);
    groundShader_.setVec3("uAmbient",  ambient);
    glBindVertexArray(groundVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
}

void Renderer::render(const Camera& cam,
                      const std::vector<std::unique_ptr<RigidBody>>& bodies,
                      const std::vector<std::unique_ptr<GPUMesh>>&   meshes,
                      const std::vector<glm::vec3>&                  colors)
{
    glClearColor(clearColor.r, clearColor.g, clearColor.b, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawGround(cam);

    meshShader_.use();
    meshShader_.setMat4("uView",     cam.viewMatrix());
    meshShader_.setMat4("uProj",     cam.projectionMatrix());
    meshShader_.setVec3("uViewPos",  cam.eye());
    meshShader_.setVec3("uLightDir", glm::normalize(lightDir));
    meshShader_.setVec3("uLightCol", lightColor);
    meshShader_.setVec3("uAmbient",  ambient);

    const std::size_t n = std::min(bodies.size(), meshes.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (!meshes[i] || !meshes[i]->valid()) continue;

        glm::mat4 model  = bodies[i]->modelMatrix();
        glm::mat3 normal = glm::transpose(glm::inverse(glm::mat3(model)));
        meshShader_.setMat4("uModel",  model);
        meshShader_.setMat3("uNormal", normal);

        glm::vec3 color = (i < colors.size()) ? colors[i] : paletteColor(i);
        meshShader_.setVec3("uBaseColor", color);

        meshes[i]->draw();
    }
}

} // namespace destruct
