// Application.cpp
#include "app/Application.h"
#include "mesh/OBJLoader.h"
#include "mesh/MeshProperties.h"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <utility>

namespace destruct {

// Lifecycle
Application::Application(Config cfg) : cfg_(std::move(cfg)) {}

Application::~Application()
{
    shutdown();
}

void Application::shutdown()
{
    gpuMeshes_.clear();   // delete GL resources while context is alive
    if (window_) {
        glfwDestroyWindow(window_);
        window_ = nullptr;
    }
    glfwTerminate();
}

bool Application::initWindow()
{
    if (!glfwInit()) {
        std::cerr << "glfwInit failed\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    window_ = glfwCreateWindow(cfg_.windowWidth,
                               cfg_.windowHeight,
                               cfg_.windowTitle.c_str(),
                               nullptr,
                               nullptr);
    if (!window_) {
        std::cerr << "glfwCreateWindow failed\n";
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // vsync on

    glfwSetWindowUserPointer(window_, this);
    glfwSetFramebufferSizeCallback(window_, &Application::onFramebufferSize);
    glfwSetMouseButtonCallback  (window_, &Application::onMouseButton);
    glfwSetCursorPosCallback    (window_, &Application::onCursorPos);
    glfwSetScrollCallback       (window_, &Application::onScroll);
    glfwSetKeyCallback          (window_, &Application::onKey);
    return true;
}

bool Application::initGL()
{
    glewExperimental = GL_TRUE;
    GLenum glewErr = glewInit();
    if (glewErr != GLEW_OK) {
        std::cerr << "glewInit failed: "
                  << reinterpret_cast<const char*>(glewGetErrorString(glewErr))
                  << "\n";
        return false;
    }
    // glewInit often leaves an GL_INVALID_ENUM queued; flush it.
    (void)glGetError();

    int fbw, fbh;
    glfwGetFramebufferSize(window_, &fbw, &fbh);
    renderer_.setViewport(camera_, fbw, fbh);

    if (!renderer_.init(cfg_.shaderDir)) {
        std::cerr << "renderer init failed\n";
        return false;
    }
    glEnable(GL_MULTISAMPLE);
    return true;
}

// Scene setup
void Application::loadSourceMesh()
{
    sourceMesh_.clear();
    bool fromObj = false;
    if (!cfg_.objPath.empty()) {
        std::string err;
        if (!loadOBJ(cfg_.objPath, sourceMesh_, &err) || sourceMesh_.empty()) {
            std::cerr << "failed to load OBJ '" << cfg_.objPath
                      << "': " << err << "\n";
            std::cerr << "falling back to default cube.\n";
            sourceMesh_ = Mesh::makeCube(0.5f);
        } else {
            fromObj = true;
        }
    } else {
        sourceMesh_ = Mesh::makeCube(0.5f);
    }

    // Normalize any OBJ-loaded mesh to a canonical size + position so the
    // camera, spawn offset, and physics tuning work uniformly across input
    // files. Target: centered on origin, max AABB dimension = 1.0 (matches
    // the default cube). The built-in cube factory is already in this form
    // so we skip normalization on that path to avoid a no-op round-trip.
    if (fromObj) {
        glm::vec3 bbMin, bbMax;
        sourceMesh_.computeAABB(bbMin, bbMax);
        const glm::vec3 center = 0.5f * (bbMin + bbMax);
        const glm::vec3 ext    = bbMax - bbMin;
        const float maxDim =
            std::max(ext.x, std::max(ext.y, ext.z));
        const float targetSize = 1.0f;
        const float scale =
            (maxDim > 1e-6f) ? (targetSize / maxDim) : 1.0f;
        for (auto& v : sourceMesh_.vertices) {
            v = (v - center) * scale;
        }
        std::cerr << "[loadSourceMesh] normalized OBJ: "
                  << "verts=" << sourceMesh_.vertices.size()
                  << " tris="  << sourceMesh_.triangles.size()
                  << " scale=" << scale << "\n";
    }

    sourceMesh_.recomputeNormals();
}

void Application::clearScene()
{
    world_.clear();
    gpuMeshes_.clear();
    bodyColors_.clear();
    recorder_.clear();
    scrubFrame_ = 0;
    initialStates_.clear();
}

void Application::buildRigidBodiesAndGPUMeshes(
    const std::vector<Mesh>& fragments)
{
    // For each fragment, compute its true center of mass (needed to place
    // the body back at its original slot in the scene after the
    // RigidBody constructor recenters the mesh) and build a rigid body.
    for (const auto& frag : fragments) {
        auto body = RigidBody::fromMesh(frag, /*density*/ 1200.0f);
        if (!body) continue;

        // The "approxCoM" from AABB above is only for placement; the
        // accurate CoM is already baked into the body. To put the body
        // back in its original world slot, we need to know the exact
        // pre-center CoM — which is what RigidBody::fromMesh used
        // internally. Recompute it here by the same method so we agree.
        MassProperties mp = computeMassProperties(frag);
        glm::vec3 trueCoM = mp.centerOfMass;

        body->position = spawnOffset_ + trueCoM;

        auto gm = std::make_unique<GPUMesh>();
        gm->uploadFromMesh(body->meshLocal);

        gpuMeshes_.push_back(std::move(gm));
        world_.addBody(std::move(body));
    }

    // Snapshot the post-build configuration so R (restart) can put every
    // fragment back exactly where it started, without re-running fracture.
    initialStates_.clear();
    initialStates_.reserve(world_.bodies.size());
    for (const auto& b : world_.bodies) {
        initialStates_.push_back({ b->position, b->orientation });
    }
}

void Application::refracture(FractureMethod method,
                             bool impactBiased,
                             const glm::vec3& impactPoint)
{
    clearScene();

    std::vector<Mesh> fragments;
    if (method == FractureMethod::Voronoi) {
        VoronoiParams params;
        params.numSeeds      = currentFragmentCount_;
        params.rngSeed       = 0; // time-seeded
        params.minSeedSeparation = 0.0f;
        if (impactBiased) {
            params.useImpactBias = true;
            // Convert world-space impact to source-mesh-local space.
            params.impactPoint   = impactPoint - spawnOffset_;
            glm::vec3 bbMin, bbMax;
            sourceMesh_.computeAABB(bbMin, bbMax);
            glm::vec3 ext = bbMax - bbMin;
            params.impactRadius = 0.25f * std::max(ext.x,
                                        std::max(ext.y, ext.z));
        }
        voronoiFracture(sourceMesh_, params, fragments);
    } else {
        UniformFractureParams up;
        int side = std::max(2,
            static_cast<int>(std::ceil(std::cbrt((double)currentFragmentCount_))));
        up.nx = up.ny = up.nz = side;
        uniformFracture(sourceMesh_, up, fragments);
    }

    buildRigidBodiesAndGPUMeshes(fragments);

    // New scene -> always wait for the user to hit Space. This matches the
    // expectation that fracturing shouldn't auto-simulate; you should see
    // the fragments sitting whole, then start the simulation deliberately.
    paused_ = true;

    std::cerr << "[refracture] method="
              << (method == FractureMethod::Voronoi ? "voronoi" : "uniform")
              << " fragments=" << world_.bodies.size() << "\n";
}

// Run loop
int Application::run()
{
    if (!initWindow()) return 1;
    if (!initGL())     return 2;

    loadSourceMesh();
    currentFragmentCount_ = cfg_.initialFragments;
    refracture(FractureMethod::Voronoi, false, glm::vec3(0.0f));

    // Nice starting camera: look at the rough scene centre.
    camera_.target   = spawnOffset_;
    camera_.distance = 4.5f;

    lastTime_ = glfwGetTime();
    lastTitleTime_ = lastTime_;
    frameCount_ = 0;

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        double now = glfwGetTime();
        float  dt  = static_cast<float>(now - lastTime_);
        lastTime_  = now;
        ++frameCount_;
        const double titleElapsed = now - lastTitleTime_;
        if (titleElapsed >= 0.25) {
            const double fps = frameCount_ / titleElapsed;
            char title[256];
            std::snprintf(title, sizeof(title), "%s - %d FPS",
                          cfg_.windowTitle.c_str(),
                          static_cast<int>(std::round(fps)));
            glfwSetWindowTitle(window_, title);
            lastTitleTime_ = now;
            frameCount_ = 0;
        }
        // Clamp dt to avoid a giant explosion on a slow frame.
        dt = std::min(dt, 1.0f / 20.0f);

        handleInput(dt);
        simulate(dt);
        renderFrame();
        glfwSwapBuffers(window_);
    }
    return 0;
}

void Application::handleInput(float /*dt*/)
{
    // Orbit / pan are event-driven (mouse callbacks). Nothing continuous
    // needed here for now.
}

void Application::simulate(float dt)
{
    if (paused_) return;

    // Step with a fixed sub-timestep for numerical stability.
    const float subDt = 1.0f / 120.0f;
    float remaining = dt;
    int safety = 8;
    while (remaining > 1e-4f && safety-- > 0) {
        float h = std::min(subDt, remaining);
        world_.step(h);
        remaining -= h;
    }

    recorder_.recordFrame(world_.bodies);
    scrubFrame_ = recorder_.frameCount() - 1;
}

void Application::renderFrame()
{
    int fbw, fbh;
    glfwGetFramebufferSize(window_, &fbw, &fbh);
    if (fbw > 0 && fbh > 0) {
        renderer_.setViewport(camera_, fbw, fbh);
    }
    renderer_.render(camera_, world_.bodies, gpuMeshes_, bodyColors_);
}

// Raycasting + explosions
namespace {

// Ray/triangle intersection (Moller-Trumbore). Returns true and sets t
// on hit (t > 0). No backface culling so we can hit inward-facing tris of
// concave fragment assemblies.
bool rayTri(const glm::vec3& o, const glm::vec3& d,
            const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
            float& tOut)
{
    const float EPS = 1e-7f;
    glm::vec3 e1 = v1 - v0;
    glm::vec3 e2 = v2 - v0;
    glm::vec3 p  = glm::cross(d, e2);
    float det    = glm::dot(e1, p);
    if (std::fabs(det) < EPS) return false;
    float invDet = 1.0f / det;
    glm::vec3 tv = o - v0;
    float u = glm::dot(tv, p) * invDet;
    if (u < 0.0f || u > 1.0f) return false;
    glm::vec3 q = glm::cross(tv, e1);
    float v     = glm::dot(d, q) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = glm::dot(e2, q) * invDet;
    if (t <= 1e-4f) return false;
    tOut = t;
    return true;
}

// Ray vs AABB slab test. Used as a broad-phase reject in pickBodyByRay.
bool rayAABB(const glm::vec3& o, const glm::vec3& d,
             const glm::vec3& bbMin, const glm::vec3& bbMax)
{
    float tMin = 0.0f;
    float tMax = std::numeric_limits<float>::infinity();
    for (int i = 0; i < 3; ++i) {
        float invD = 1.0f / (std::fabs(d[i]) > 1e-20f ? d[i] : 1e-20f);
        float t1 = (bbMin[i] - o[i]) * invD;
        float t2 = (bbMax[i] - o[i]) * invD;
        if (t1 > t2) std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax) return false;
    }
    return true;
}

} // namespace

int Application::pickBodyByRay(double px, double py, glm::vec3& outHit) const
{
    glm::vec3 ro, rd;
    camera_.getRayFromScreen(px, py, ro, rd);

    int   bestIdx = -1;
    float bestT   = std::numeric_limits<float>::infinity();

    for (std::size_t i = 0; i < world_.bodies.size(); ++i) {
        const auto& b = world_.bodies[i];

        // Transform the ray into body space to avoid transforming every vertex.
        glm::vec3 roBody = b->worldToBody(ro);
        glm::vec3 rdBody = glm::conjugate(b->orientation) * rd;

        if (!rayAABB(roBody, rdBody, b->aabbMin, b->aabbMax)) continue;

        const auto& verts = b->meshLocal.vertices;
        for (const auto& tri : b->meshLocal.triangles) {
            float t;
            if (rayTri(roBody, rdBody,
                       verts[tri.a], verts[tri.b], verts[tri.c], t))
            {
                if (t < bestT) {
                    bestT = t;
                    bestIdx = static_cast<int>(i);
                }
            }
        }
    }
    if (bestIdx < 0) return -1;
    outHit = ro + rd * bestT;
    return bestIdx;
}

void Application::applyExplosion(const glm::vec3& impactPoint, float strength)
{
    for (auto& b : world_.bodies) {
        glm::vec3 to = b->position - impactPoint;
        float d2 = glm::dot(to, to);
        if (d2 < 1e-8f) continue;
        float d = std::sqrt(d2);
        glm::vec3 dir = to / d;
        // Inverse-square with a floor so close bodies don't get infinite impulse.
        float falloff = strength / (0.1f + d * d);
        glm::vec3 impulse = dir * falloff * b->mass;
        b->applyImpulseAtPoint(impulse, impactPoint);
    }
}

// GLFW trampolines
void Application::onFramebufferSize(GLFWwindow* w, int /*wid*/, int /*hei*/)
{
    auto* self = static_cast<Application*>(glfwGetWindowUserPointer(w));
    if (!self) return;
    int fbw, fbh;
    glfwGetFramebufferSize(w, &fbw, &fbh);
    self->renderer_.setViewport(self->camera_, fbw, fbh);
}
void Application::onMouseButton(GLFWwindow* w, int b, int act, int mods)
{
    auto* self = static_cast<Application*>(glfwGetWindowUserPointer(w));
    if (self) self->handleMouseButton(b, act, mods);
}
void Application::onCursorPos(GLFWwindow* w, double x, double y)
{
    auto* self = static_cast<Application*>(glfwGetWindowUserPointer(w));
    if (self) self->handleCursorPos(x, y);
}
void Application::onScroll(GLFWwindow* w, double dx, double dy)
{
    auto* self = static_cast<Application*>(glfwGetWindowUserPointer(w));
    if (self) self->handleScroll(dx, dy);
}
void Application::onKey(GLFWwindow* w, int k, int s, int act, int mods)
{
    auto* self = static_cast<Application*>(glfwGetWindowUserPointer(w));
    if (self) self->handleKey(k, s, act, mods);
}

void Application::handleMouseButton(int b, int act, int /*mods*/)
{
    double x, y;
    glfwGetCursorPos(window_, &x, &y);
    lastCursorX_ = x;
    lastCursorY_ = y;

    if (b == GLFW_MOUSE_BUTTON_LEFT)   leftDown_   = (act == GLFW_PRESS);
    if (b == GLFW_MOUSE_BUTTON_MIDDLE) middleDown_ = (act == GLFW_PRESS);

    if (b == GLFW_MOUSE_BUTTON_RIGHT && act == GLFW_PRESS) {
        // Raycast-driven impact: refracture around the hit, then apply an
        // explosion impulse. refracture() pauses the sim; the explosion
        // impulse is held in the bodies' velocities (they're awake, just
        // not integrating) until the user hits Space to play.
        glm::vec3 hit;
        int idx = pickBodyByRay(x, y, hit);
        if (idx >= 0) {
            refracture(method_, /*impactBiased*/ true, hit);
            applyExplosion(hit, 4.5f);
        }
    }
}

void Application::handleCursorPos(double x, double y)
{
    const double dx = x - lastCursorX_;
    const double dy = y - lastCursorY_;
    lastCursorX_ = x;
    lastCursorY_ = y;

    if (leftDown_) {
        camera_.orbit(static_cast<float>(dx) * 0.005f,
                      static_cast<float>(dy) * 0.005f);
    } else if (middleDown_) {
        camera_.pan(static_cast<float>(dx), static_cast<float>(dy));
    }
}

void Application::handleScroll(double /*dx*/, double dy)
{
    camera_.dolly(static_cast<float>(dy));
}

void Application::handleKey(int k, int /*s*/, int act, int /*mods*/)
{
    if (act != GLFW_PRESS && act != GLFW_REPEAT) return;

    switch (k) {
    case GLFW_KEY_ESCAPE:
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
        break;
    case GLFW_KEY_R:
        // Restart: restore every fragment to its initial placement, zero
        // velocities, wake all, clear recorded timeline, and pause so the
        // user explicitly hits Space to re-run the same scenario.
        for (std::size_t i = 0;
             i < std::min(world_.bodies.size(), initialStates_.size());
             ++i)
        {
            auto& b = world_.bodies[i];
            b->position        = initialStates_[i].position;
            b->orientation     = initialStates_[i].orientation;
            b->linearVelocity  = glm::vec3(0.0f);
            b->angularVelocity = glm::vec3(0.0f);
            b->sleeping        = false;
            b->sleepTimer      = 0.0f;
            b->refreshInertiaWorld();
        }
        recorder_.clear();
        scrubFrame_ = 0;
        paused_ = true;
        break;
    case GLFW_KEY_F:
        // Refracture the pristine mesh with fresh seeds (was R in the
        // older build).
        refracture(method_, false, glm::vec3(0.0f));
        break;
    case GLFW_KEY_V:
        method_ = FractureMethod::Voronoi;
        refracture(method_, false, glm::vec3(0.0f));
        break;
    case GLFW_KEY_U:
        method_ = FractureMethod::Uniform;
        refracture(method_, false, glm::vec3(0.0f));
        break;
    case GLFW_KEY_SPACE:
        paused_ = !paused_;
        // Waking on resume is belt-and-suspenders: any impulse wakes a
        // body, but the user might also expect a slight "kick" on unpause
        // even without explicit forces. Cheap to wake all here.
        if (!paused_) world_.wakeAll();
        break;
    case GLFW_KEY_LEFT:
        if (paused_ && scrubFrame_ > 0) {
            --scrubFrame_;
            recorder_.seek(scrubFrame_, world_.bodies);
        }
        break;
    case GLFW_KEY_RIGHT:
        if (paused_ && scrubFrame_ + 1 < recorder_.frameCount()) {
            ++scrubFrame_;
            recorder_.seek(scrubFrame_, world_.bodies);
        }
        break;
    case GLFW_KEY_UP:
        currentFragmentCount_ = std::min(currentFragmentCount_ + 5, 200);
        break;
    case GLFW_KEY_DOWN:
        currentFragmentCount_ = std::max(currentFragmentCount_ - 5, 4);
        break;
    case GLFW_KEY_G:
        // "Pop" -- upward impulse to every body. applyImpulseAtPoint
        // already wakes each body, so sleeping fragments participate too.
        for (auto& b : world_.bodies) {
            b->applyImpulseAtPoint(glm::vec3(0.0f, 3.5f * b->mass, 0.0f),
                                   b->position);
        }
        break;
    default: break;
    }
}

} // namespace destruct
