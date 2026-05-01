#pragma once
// Application.h
//
// Top-level app: owns the GLFW window, input state, the simulation
// components (fracture params, rigid bodies, physics world, recorder),
// and the renderer. `run()` is the main loop.
//
// Controls:
//   * Mouse left drag  : orbit camera
//   * Mouse middle drag: pan camera
//   * Mouse wheel      : zoom
//   * Mouse right click: raycast impact -> re-fracture + impulse
//   * Space            : play / pause (simulation starts paused)
//   * R                : restart — reset fragments to initial positions (pauses)
//   * F                : refracture the pristine mesh with fresh seeds
//   * V                : switch to Voronoi fracture + refracture
//   * U                : switch to uniform-grid fracture + refracture
//   * Left / Right     : scrub playback (when paused)
//   * Up / Down        : increase / decrease fragment count + refracture
//   * G                : pop — upward impulse on every fragment
//   * Esc              : quit

#include "fracture/VoronoiFracture.h"
#include "fracture/UniformFracture.h"
#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "render/Renderer.h"
#include "render/Camera.h"
#include "render/GPUMesh.h"
#include "mesh/Mesh.h"
#include "app/TimestepRecorder.h"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

struct GLFWwindow;

namespace destruct {

class Application {
public:
    // Configuration passed from main().
    struct Config {
        int         windowWidth  = 1280;
        int         windowHeight = 720;
        std::string windowTitle  = "CS184 - Interactive Geometry Destruction";
        std::string shaderDir    = "shaders";
        std::string objPath;       // empty => use default cube mesh
        int         initialFragments = 20;
    };

    explicit Application(Config cfg);
    ~Application();
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    // Blocking main loop. Returns 0 on clean exit, non-zero on init failure.
    int run();

private:
    // --- Lifecycle ---
    bool initWindow();
    bool initGL();
    void shutdown();

    // --- Scene ---
    enum class FractureMethod { Voronoi, Uniform };

    void loadSourceMesh();
    void refracture(FractureMethod method,
                    bool impactBiased,
                    const glm::vec3& impactPoint);
    void syncUniformGridToFragmentCount();
    void clearScene();
    void buildRigidBodiesAndGPUMeshes(const std::vector<Mesh>& fragments);

    // --- Frame ---
    void handleInput(float dt);
    void simulate(float dt);
    void renderFrame();

    // --- Impact raycast (aspirational feature) ---
    // Find the first body hit by a camera ray from screen coords
    // (px, py) and return the world-space hit point in `outHit`.
    // Returns the index of the hit body or -1.
    int pickBodyByRay(double px, double py, glm::vec3& outHit) const;
    // Add an explosive impulse at `impactPoint` to all bodies near it,
    // falling off with distance.
    void applyExplosion(const glm::vec3& impactPoint, float strength);

    // --- GLFW callbacks (static trampolines) ---
    static void onFramebufferSize(GLFWwindow* w, int wid, int hei);
    static void onMouseButton(GLFWwindow* w, int b, int act, int mods);
    static void onCursorPos(GLFWwindow* w, double x, double y);
    static void onScroll(GLFWwindow* w, double dx, double dy);
    static void onKey(GLFWwindow* w, int k, int s, int act, int mods);

    // Instance dispatch.
    void handleMouseButton(int b, int act, int mods);
    void handleCursorPos(double x, double y);
    void handleScroll(double dx, double dy);
    void handleKey(int k, int s, int act, int mods);

private:
    Config      cfg_;
    GLFWwindow* window_ = nullptr;

    // Source (pristine) mesh, plus a unit transform that positions it
    // at a comfortable start location.
    Mesh       sourceMesh_;
    glm::vec3  spawnOffset_{0.0f, 2.0f, 0.0f};

    // Runtime state.
    PhysicsWorld           world_;
    std::vector<std::unique_ptr<GPUMesh>> gpuMeshes_;
    std::vector<glm::vec3>                bodyColors_;
    TimestepRecorder       recorder_;
    std::size_t            scrubFrame_ = 0;

    Renderer   renderer_;
    Camera     camera_;

    // Input state.
    double lastCursorX_ = 0.0;
    double lastCursorY_ = 0.0;
    bool   leftDown_    = false;
    bool   middleDown_  = false;

    // Simulation state flags.
    bool paused_           = true;   // start paused; user hits Space to run
    bool showUniform_      = false;
    FractureMethod method_ = FractureMethod::Voronoi;
    int    currentFragmentCount_ = 20;
    int    uniformNx_ = 3;
    int    uniformNy_ = 3;
    int    uniformNz_ = 3;

    // Initial post-fracture transforms — used by Restart (R) to reset the
    // scene to its starting configuration without regenerating fragments.
    std::vector<RBState> initialStates_;

    // Timing.
    double lastTime_       = 0.0;
    double lastTitleTime_  = 0.0;
    int    frameCount_     = 0;
};

} // namespace destruct
