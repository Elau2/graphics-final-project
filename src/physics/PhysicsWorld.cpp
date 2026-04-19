// PhysicsWorld.cpp
#include "physics/PhysicsWorld.h"

#include <glm/gtc/quaternion.hpp>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace destruct {

// Scene management
RigidBody* PhysicsWorld::addBody(std::unique_ptr<RigidBody> body)
{
    RigidBody* raw = body.get();
    bodies.emplace_back(std::move(body));
    return raw;
}

void PhysicsWorld::clear()
{
    bodies.clear();
}

void PhysicsWorld::wakeAll()
{
    for (auto& b : bodies) {
        b->sleeping   = false;
        b->sleepTimer = 0.0f;
    }
}

// Step pipeline
void PhysicsWorld::step(float dt)
{
    if (dt <= 0.0f) return;

    // 1. Apply gravity; integrate v from a = F/m (semi-implicit).
    integrateForces(dt);

    // 2. Integrate positions / orientations from the new velocities.
    for (auto& b : bodies) {
        if (b->isStatic() || b->sleeping) continue;

        b->position += b->linearVelocity * dt;

        // Quaternion derivative: dq/dt = 0.5 * omega * q
        glm::quat wq(0.0f, b->angularVelocity);
        b->orientation += 0.5f * dt * (wq * b->orientation);
        b->orientation  = glm::normalize(b->orientation);

        b->refreshInertiaWorld();
    }

    // 3. Collision detection + impulse resolution.
    collideGround();
    collideBodies();

    // 4. Put slow-moving bodies to sleep so gravity + restitution don't
    //    perpetually nudge resting piles.
    updateSleep(dt);
}

void PhysicsWorld::integrateForces(float dt)
{
    const float linDampFactor = std::max(0.0f, 1.0f - linearDamping  * dt);
    const float angDampFactor = std::max(0.0f, 1.0f - angularDamping * dt);

    for (auto& b : bodies) {
        if (b->isStatic() || b->sleeping) {
            // Sleeping bodies don't accumulate gravity and don't integrate.
            // Their velocity is already zero, and keeping it that way is the
            // whole point of sleep.
            b->forceAccum  = glm::vec3(0.0f);
            b->torqueAccum = glm::vec3(0.0f);
            continue;
        }

        // gravity
        b->forceAccum += gravity * b->mass;

        // v <- v + (F/m) dt
        b->linearVelocity  += b->invMass * b->forceAccum * dt;
        // omega <- omega + I^-1 tau dt
        b->angularVelocity += b->invIworld * b->torqueAccum * dt;

        // linear / angular damping
        b->linearVelocity  *= linDampFactor;
        b->angularVelocity *= angDampFactor;

        // Clear accumulators for next step.
        b->forceAccum  = glm::vec3(0.0f);
        b->torqueAccum = glm::vec3(0.0f);

        // Refresh invIworld so collision resolution sees the current
        // orientation's inertia.
        b->refreshInertiaWorld();
    }
}

// Contact resolution
// We split "resolve" into two separate steps so that the multi-contact ground
// solver can apply a velocity impulse at every penetrating vertex (for stable
// resting contact) while applying positional correction only *once* per body
// (at the deepest point), avoiding over-correction that would fling the body
// upward.
namespace {

// Apply only the velocity impulse (normal + Coulomb-clamped friction) at a
// single contact point. `n` points from B into A (so for ground contacts
// n = +Y). Does nothing if the contact is separating.
void applyVelocityImpulse(RigidBody* a, RigidBody* b,
                          const glm::vec3& worldPoint,
                          const glm::vec3& n,
                          float restitutionThreshold)
{
    const glm::vec3 ra = worldPoint - a->position;
    const glm::vec3 va = a->pointVelocityWorld(worldPoint);
    glm::vec3 vRel = va;
    if (b) vRel -= b->pointVelocityWorld(worldPoint);

    const float vn = glm::dot(vRel, n);
    if (vn >= 0.0f) return; // separating (or exactly tangential): no impulse

    // Baraff-style restitution threshold: below this approach speed, treat
    // the collision as perfectly plastic (e = 0). This is the single change
    // that eliminates the "perpetual rattle" on resting bodies: without it,
    // gravity's per-step velocity injection gets bounced back every frame.
    float e = b ? 0.5f * (a->restitution + b->restitution) : a->restitution;
    if (-vn < restitutionThreshold) e = 0.0f;

    const float invMassSum = a->invMass + (b ? b->invMass : 0.0f);

    glm::vec3 raCrossN = glm::cross(ra, n);
    glm::vec3 angA     = a->invIworld * raCrossN;
    float     angTerm  = glm::dot(glm::cross(angA, ra), n);

    float angB = 0.0f;
    glm::vec3 rb(0.0f);
    if (b) {
        rb = worldPoint - b->position;
        glm::vec3 rbCrossN = glm::cross(rb, n);
        glm::vec3 angBvec  = b->invIworld * rbCrossN;
        angB = glm::dot(glm::cross(angBvec, rb), n);
    }

    const float denom = invMassSum + angTerm + angB;
    if (denom <= 1e-12f) return;

    float jn = -(1.0f + e) * vn / denom;
    if (jn <= 0.0f) return;

    glm::vec3 impulseN = jn * n;
    a->applyImpulseAtPoint( impulseN, worldPoint);
    if (b) b->applyImpulseAtPoint(-impulseN, worldPoint);

    // --- Friction ---
    glm::vec3 va2 = a->pointVelocityWorld(worldPoint);
    glm::vec3 vRel2 = va2;
    if (b) vRel2 -= b->pointVelocityWorld(worldPoint);
    glm::vec3 vt = vRel2 - glm::dot(vRel2, n) * n;

    const float vtLen = glm::length(vt);
    if (vtLen > 1e-6f) {
        glm::vec3 t = vt / vtLen;

        glm::vec3 raCrossT = glm::cross(ra, t);
        glm::vec3 angAt    = a->invIworld * raCrossT;
        float angTermT     = glm::dot(glm::cross(angAt, ra), t);
        float angBt = 0.0f;
        if (b) {
            glm::vec3 rbCrossT = glm::cross(rb, t);
            glm::vec3 angBtvec = b->invIworld * rbCrossT;
            angBt = glm::dot(glm::cross(angBtvec, rb), t);
        }
        float denomT = invMassSum + angTermT + angBt;
        if (denomT > 1e-12f) {
            float jt = -glm::dot(vRel2, t) / denomT;

            const float mu = b ? 0.5f * (a->friction + b->friction)
                               : a->friction;
            // Coulomb friction cone clamp against the normal impulse magnitude.
            jt = std::max(-mu * jn, std::min(mu * jn, jt));

            glm::vec3 impulseT = jt * t;
            a->applyImpulseAtPoint( impulseT, worldPoint);
            if (b) b->applyImpulseAtPoint(-impulseT, worldPoint);
        }
    }
}

// Positional correction (Baumgarte). Applied once per body per step, not
// once per contact, to avoid over-correction when many vertices penetrate.
void applyPositionalCorrection(RigidBody* a, RigidBody* b,
                               const glm::vec3& n, float penetration,
                               float slop, float percent)
{
    if (penetration <= slop) return;
    const float invMassSum = a->invMass + (b ? b->invMass : 0.0f);
    if (invMassSum <= 0.0f) return;
    const glm::vec3 correction =
        (std::max(penetration - slop, 0.0f) / invMassSum) * percent * n;
    a->position += a->invMass * correction;
    if (b) b->position -= b->invMass * correction;
}

} // anonymous namespace

// Ground collision: multi-contact impulse, single positional correction
void PhysicsWorld::collideGround()
{
    const glm::vec3 n(0.0f, 1.0f, 0.0f);

    for (auto& bPtr : bodies) {
        RigidBody* b = bPtr.get();
        if (b->isStatic()) continue;

        // Broad-phase: if the bounding sphere is entirely above ground,
        // skip the per-vertex test.
        if (b->position.y - b->boundingRadius > groundY) continue;

        // Pass 1: apply a velocity impulse at every penetrating vertex.
        // This is crucial for resting stability — a cube on its face has
        // four penetrating corners, and resolving only one leaves three
        // unresolved, which produces the "never settles, always pivots"
        // behaviour. Iterating all of them kills tangential rotation from
        // the first step.
        float    maxPen   = 0.0f;
        glm::vec3 deepest{0.0f};
        bool     hasDeep  = false;

        // NOTE: if `b` is sleeping, its velocity is zero, so vn inside
        // applyVelocityImpulse will be zero and the impulse returns
        // without touching the body. That's exactly what we want: a
        // sleeping body sitting on the ground does nothing each frame,
        // and stays asleep.
        for (const auto& vLocal : b->meshLocal.vertices) {
            const glm::vec3 w = b->bodyToWorld(vLocal);
            const float pen = groundY - w.y;
            if (pen > 0.0f) {
                applyVelocityImpulse(b, nullptr, w, n, restitutionThreshold);
                if (pen > maxPen) { maxPen = pen; deepest = w; hasDeep = true; }
            }
        }

        // Pass 2: a single positional correction at the deepest point, so
        // the body doesn't get flung upward by N corrections for N vertices.
        if (hasDeep) {
            applyPositionalCorrection(b, nullptr, n, maxPen,
                                      positionSlop, positionPercent);
        }
    }
}

// Body-body collision: single deepest contact per pair (unchanged behaviour)
void PhysicsWorld::collideBodies()
{
    // O(N^2) broad-phase: acceptable at the fragment counts we target.
    // A uniform grid could replace this if we ever want N in the thousands.
    const std::size_t N = bodies.size();
    for (std::size_t i = 0; i < N; ++i) {
        RigidBody* a = bodies[i].get();
        if (a->isStatic()) continue;
        for (std::size_t j = i + 1; j < N; ++j) {
            RigidBody* b = bodies[j].get();

            // Two sleeping bodies can't possibly produce new contact dynamics.
            if (a->sleeping && b->sleeping) continue;

            // Bounding-sphere reject.
            glm::vec3 delta = a->position - b->position;
            float rSum = a->boundingRadius + b->boundingRadius;
            if (glm::dot(delta, delta) > rSum * rSum) continue;

            // Narrow phase: test every vertex of A against B's body-space
            // AABB (and vice versa), keeping the deepest penetration as a
            // single contact. Rough but adequate for the small fragment
            // sizes this project targets.
            float deepestPen = 0.0f;
            glm::vec3 bestWorld(0.0f);
            glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
            bool have = false;

            auto probe = [&](RigidBody* P, RigidBody* Q) {
                for (const auto& vLocal : P->meshLocal.vertices) {
                    glm::vec3 w = P->bodyToWorld(vLocal);
                    glm::vec3 qLocal = Q->worldToBody(w);
                    if (qLocal.x < Q->aabbMin.x || qLocal.x > Q->aabbMax.x) continue;
                    if (qLocal.y < Q->aabbMin.y || qLocal.y > Q->aabbMax.y) continue;
                    if (qLocal.z < Q->aabbMin.z || qLocal.z > Q->aabbMax.z) continue;

                    // Penetration = distance to nearest face of the AABB.
                    float dx1 = qLocal.x - Q->aabbMin.x;
                    float dx2 = Q->aabbMax.x - qLocal.x;
                    float dy1 = qLocal.y - Q->aabbMin.y;
                    float dy2 = Q->aabbMax.y - qLocal.y;
                    float dz1 = qLocal.z - Q->aabbMin.z;
                    float dz2 = Q->aabbMax.z - qLocal.z;

                    float minD = dx1;  glm::vec3 nLocal(-1, 0, 0);
                    if (dx2 < minD) { minD = dx2; nLocal = glm::vec3( 1, 0, 0); }
                    if (dy1 < minD) { minD = dy1; nLocal = glm::vec3( 0,-1, 0); }
                    if (dy2 < minD) { minD = dy2; nLocal = glm::vec3( 0, 1, 0); }
                    if (dz1 < minD) { minD = dz1; nLocal = glm::vec3( 0, 0,-1); }
                    if (dz2 < minD) { minD = dz2; nLocal = glm::vec3( 0, 0, 1); }

                    if (minD > deepestPen) {
                        deepestPen = minD;
                        bestWorld  = w;
                        bestNormal = glm::normalize(Q->orientation * (-nLocal));
                        have = true;
                    }
                }
            };

            probe(a, b);
            probe(b, a);

            if (have && deepestPen > 0.0f) {
                applyVelocityImpulse(a, b, bestWorld, bestNormal,
                                     restitutionThreshold);
                applyPositionalCorrection(a, b, bestNormal, deepestPen,
                                          positionSlop, positionPercent);
            }
        }
    }
}

// Sleep latching
void PhysicsWorld::updateSleep(float dt)
{
    const float linSq = sleepLinear  * sleepLinear;
    const float angSq = sleepAngular * sleepAngular;

    for (auto& b : bodies) {
        if (b->isStatic() || b->sleeping) continue;

        const float linKE2 = glm::dot(b->linearVelocity,  b->linearVelocity);
        const float angKE2 = glm::dot(b->angularVelocity, b->angularVelocity);

        if (linKE2 < linSq && angKE2 < angSq) {
            b->sleepTimer += dt;
            if (b->sleepTimer >= sleepTime) {
                // Latch fully asleep: zero velocity so no residual drift,
                // and future step()s skip integration entirely.
                b->sleeping        = true;
                b->linearVelocity  = glm::vec3(0.0f);
                b->angularVelocity = glm::vec3(0.0f);
            }
        } else {
            b->sleepTimer = 0.0f;
        }
    }
}

} // namespace destruct
