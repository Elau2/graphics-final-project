#pragma once
// PhysicsWorld.h
//
// A minimal custom rigid-body dynamics world. It owns a list of rigid
// bodies and integrates them each step. The world is deliberately small
// and self-contained (no Bullet dependency) so the project demonstrates
// our own physics implementation, as promised in the proposal.
//
// Collisions handled:
//   * Body <-> infinite ground plane at y = groundY. Per-vertex penetration
//     check; impulse resolution with restitution + Coulomb friction.
//   * Body <-> body broad-phase: bounding-sphere overlap. Narrow-phase:
//     vertex-of-A-inside-AABB-of-B sampling. Simple but adequate for the
//     visual goals of the project (piles of fragments settling). Collision
//     resolution uses the standard impulse-based formulation with
//     restitution and friction clamped by the Coulomb cone.
//
// Integration: semi-implicit Euler. Angular velocity advances the
// orientation via a first-order quaternion derivative:
//     q' = q + (dt/2) * omega_quat * q
// followed by unit-renormalisation.
//
// The world exposes a `step(dt)` entry point and the body list for the
// renderer / recorder to read out transforms.
 
#include "physics/RigidBody.h"
 
#include <glm/glm.hpp>
#include <array>
#include <memory>
#include <vector>
 
namespace destruct {
 
// Shared between PhysicsWorld's private implementation and CachedPair.
// Defined here (not in the .cpp anonymous namespace) so the header can
// reference it in CachedPair without a forward declaration.
struct Contact {
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    float penetration = 0.0f;
};
 
class PhysicsWorld {
public:
    glm::vec3 gravity{0.0f, -9.81f, 0.0f};
    float     groundY = 0.0f;
 
    // Positional correction ("Baumgarte") slop and factor. Values chosen
    // to keep stacks from jittering while still resolving deep penetration.
    float positionSlop     = 0.001f;
    float positionPercent  = 0.4f;
 
    // Linear / angular damping applied each step to keep things stable.
    float linearDamping  = 0.2f;
    float angularDamping = 0.4f;
 
    // Baraff-style restitution threshold: contacts with approach speed
    // below this are treated as perfectly plastic (e = 0). Without this,
    // gravity + a non-zero restitution create a perpetual bounce loop on
    // resting contacts and the simulation never settles.
    float restitutionThreshold = 1.0f; // m/s
 
    // Sleep thresholds. A body whose linear AND angular speeds stay below
    // these for `sleepTime` seconds is frozen. Thresholds must sit above
    // the per-step churn |g|*dt that gravity injects between resolutions,
    // or bodies at rest will never qualify.
    float sleepLinear  = 0.12f;  // m/s
    float sleepAngular = 0.25f;  // rad/s
    float sleepTime    = 0.5f;   // seconds below threshold before sleeping
 
    std::vector<std::unique_ptr<RigidBody>> bodies;
 
    PhysicsWorld() = default;
 
    // Take ownership of a body and return a raw pointer for the caller
    // to refer to it (e.g. for rendering).
    RigidBody* addBody(std::unique_ptr<RigidBody> body);
 
    // Remove all bodies.
    void clear();
 
    // Wake all bodies. Useful after user-visible changes (restart, refracture,
    // applied explosion) that should guarantee immediate response.
    void wakeAll();
 
    // Advance the simulation by `dt` seconds. Typical values: 1/60 to 1/120.
    void step(float dt);
 
private:
    void integrateForces(float dt);
    void collideGround();
    void updateSleep(float dt);
    void collideBodiesDetect();
    void collideBodiesResolve();
 
    struct CachedPair {
        RigidBody* a;
        RigidBody* b;
        int contactCount;
        std::array<Contact, 8> contacts;
    };
 
    std::vector<CachedPair> cachedPairs;
};
 
} // namespace destruct