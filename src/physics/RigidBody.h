#pragma once
// RigidBody.h
//
// One rigid body = one fragment. It owns its local-space mesh (never
// re-generated), its mass-frame inertia tensor, and a dynamic transform
// (position + orientation + linear / angular velocity).
//
// The orientation is a unit quaternion. Integration uses semi-implicit
// Euler on linear velocity and a quaternion derivative on angular
// velocity, followed by a unit renormalisation to keep the quaternion
// well-behaved over long simulations.
//
// A "static" body has infinite mass (represented by invMass == 0) and
// never moves. This is used for the ground plane via a simpler code
// path in PhysicsWorld rather than as a true RigidBody.

#include "mesh/Mesh.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

namespace destruct {

class RigidBody {
public:
    // --- Geometry (local/body frame) ---
    // The mesh is stored with its center of mass at the origin of the
    // body frame. We subtract the mesh's CoM from its vertices at
    // construction so that position/orientation directly represent the
    // body's CoM transform (simplifies collision and integration).
    Mesh meshLocal;

    // AABB of `meshLocal` in body frame. Cached for broad-phase tests.
    glm::vec3 aabbMin{0.0f};
    glm::vec3 aabbMax{0.0f};
    float     boundingRadius = 0.0f; // sphere bound around origin

    // --- Mass properties ---
    float     mass    = 1.0f;  // kg
    float     invMass = 1.0f;  // 0 for static / infinite-mass
    glm::mat3 Ibody{1.0f};     // body-frame inertia tensor
    glm::mat3 invIbody{1.0f};
    glm::mat3 invIworld{1.0f}; // refreshed every step

    // --- State ---
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f}; // identity
    glm::vec3 linearVelocity{0.0f};
    glm::vec3 angularVelocity{0.0f};

    // --- Per-step accumulators ---
    glm::vec3 forceAccum{0.0f};
    glm::vec3 torqueAccum{0.0f};

    // Material params (used by PhysicsWorld for collision resolution).
    float restitution = 0.25f;
    float friction    = 0.45f;

    // --- Sleeping ---
    // A body whose linear + angular speeds stay below PhysicsWorld's sleep
    // thresholds for `sleepTime` seconds is put to sleep: its velocities are
    // zeroed and integration is skipped until an impulse wakes it. This
    // prevents the perpetual gravity/bounce churn that makes resting piles
    // look like they're vibrating. See PhysicsWorld::updateSleep.
    bool  sleeping   = false;
    float sleepTimer = 0.0f; // seconds spent below speed threshold

    RigidBody() = default;

    // Build from a mesh + density. Computes volume, recentres mesh so CoM
    // is at the body origin, and caches body inertia. Density = kg / m^3
    // (in world units). If the mesh volume is non-positive (degenerate /
    // inside-out), the body is flagged static to avoid NaNs downstream.
    static std::unique_ptr<RigidBody>
    fromMesh(Mesh mesh, float density = 1000.0f);

    bool isStatic() const { return invMass == 0.0f; }

    // Apply a force/torque for the current step (cleared after integration).
    void applyForce(const glm::vec3& f)                          { forceAccum  += f; }
    void applyTorque(const glm::vec3& t)                         { torqueAccum += t; }
    void applyForceAtPoint(const glm::vec3& f, const glm::vec3& worldPoint);

    // Apply an instantaneous impulse at a world-space point. Updates
    // linear and angular velocity directly; bypasses the force accumulators.
    void applyImpulseAtPoint(const glm::vec3& impulse,
                             const glm::vec3& worldPoint);

    // Helpers: transform between body and world frames.
    glm::vec3 bodyToWorld(const glm::vec3& pBody) const;
    glm::vec3 worldToBody(const glm::vec3& pWorld) const;
    glm::vec3 pointVelocityWorld(const glm::vec3& pWorld) const;

    // Recompute invIworld = R * invIbody * R^T. Call once per step
    // (PhysicsWorld handles this).
    void refreshInertiaWorld();

    // 4x4 model matrix for rendering.
    glm::mat4 modelMatrix() const;
};

} // namespace destruct
