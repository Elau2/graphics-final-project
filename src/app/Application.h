#pragma once
// Application.h
 
#include "fracture/VoronoiFracture.h"
#include "fracture/UniformFracture.h"
#include "physics/PhysicsWorld.h"
#include "physics/RigidBody.h"
#include "render/Renderer.h"
#include "render/Camera.h"
#include "render/GPUMesh.h"
#include "mesh/Mesh.h"
#include "app/TimestepRecorder.h"
#include "metrics/FractureMetrics.h"
#include "metrics/MetricsOverlay.h"
#include "metrics/FractureMetrics.h"
#include "metrics/MetricsOverlay.h"
 
#include <glm/glm.hpp>
#include <array>
#include <memory>
#include <string>
#include <vector>
 
struct GLFWwindow;
 
namespace destruct {
 
class Application {
public:
    struct Config {
        int         windowWidth  = 1280;
        int         windowHeight = 720;
        std::string windowTitle  = "CS184 - Interactive Geometry Destruction";
        std::string shaderDir    = "shaders";
        std::string objPath;
        int         initialFragments = 20;
    };
 
    explicit Application(Config cfg);
    ~Application();
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;
 
    int run();
 
private:
    bool initWindow();
    bool initGL();
    void shutdown();
 
    enum class FractureMethod { Voronoi, Uniform };
 
    void loadSourceMesh();
    void refracture(FractureMethod method,
                    bool impactBiased,
                    const glm::vec3& impactPoint);
    void syncUniformGridToFragmentCount();
    void clearScene();
    void buildRigidBodiesAndGPUMeshes(const std::vector<Mesh>& fragments);
 
    void handleInput(float dt);
    void simulate(float dt);
    void renderFrame();
 
    int  pickBodyByRay(double px, double py, glm::vec3& outHit) const;
    void applyExplosion(const glm::vec3& impactPoint, float strength);
 
    static void onFramebufferSize(GLFWwindow* w, int wid, int hei);
    static void onMouseButton(GLFWwindow* w, int b, int act, int mods);
    static void onCursorPos(GLFWwindow* w, double x, double y);
    static void onScroll(GLFWwindow* w, double dx, double dy);
    static void onKey(GLFWwindow* w, int k, int s, int act, int mods);
 
    void handleMouseButton(int b, int act, int mods);
    void handleCursorPos(double x, double y);
    void handleScroll(double dx, double dy);
    void handleKey(int k, int s, int act, int mods);
 
private:
    Config      cfg_;
    GLFWwindow* window_ = nullptr;
 
    Mesh       sourceMesh_;
    glm::vec3  spawnOffset_{0.0f, 2.0f, 0.0f};
 
    PhysicsWorld                          world_;
    std::vector<std::unique_ptr<GPUMesh>> gpuMeshes_;
    std::vector<glm::vec3>                bodyColors_;
    TimestepRecorder                      recorder_;
    std::size_t                           scrubFrame_ = 0;
 
    Renderer   renderer_;
    Camera     camera_;
 
    double lastCursorX_ = 0.0;
    double lastCursorY_ = 0.0;
    bool   leftDown_    = false;
    bool   middleDown_  = false;
 
    bool paused_           = true;
    bool showUniform_      = false;
    FractureMethod method_ = FractureMethod::Voronoi;
    int    currentFragmentCount_ = 20;
    int    uniformNx_ = 3;
    int    uniformNy_ = 3;
    int    uniformNz_ = 3;
 
    std::vector<RBState> initialStates_;
 
    double lastTime_      = 0.0;
    double lastTitleTime_ = 0.0;
    int    frameCount_    = 0;
 
    // Fracture + physics success metrics (displayed as on-screen overlay).
    FractureMetrics             metrics_;
    FractureMetrics::Thresholds metricThresholds_;
    bool                        settling_ = false;
};
 
} // namespace destruct