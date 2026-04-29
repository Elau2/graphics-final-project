// PhysicsWorld.cpp
#include "physics/PhysicsWorld.h"

#include <glm/gtc/quaternion.hpp>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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
    constexpr int kBodyCollisionIterations = 4;
    for (int i = 0; i < kBodyCollisionIterations; ++i) {
        collideBodies();
    }

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

glm::vec3 closestPointOnTriangle(const glm::vec3& p,
                                 const glm::vec3& a,
                                 const glm::vec3& b,
                                 const glm::vec3& c)
{
    const glm::vec3 ab = b - a;
    const glm::vec3 ac = c - a;
    const glm::vec3 ap = p - a;
    const float d1 = glm::dot(ab, ap);
    const float d2 = glm::dot(ac, ap);
    if (d1 <= 0.0f && d2 <= 0.0f) return a;

    const glm::vec3 bp = p - b;
    const float d3 = glm::dot(ab, bp);
    const float d4 = glm::dot(ac, bp);
    if (d3 >= 0.0f && d4 <= d3) return b;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        const float v = d1 / (d1 - d3);
        return a + v * ab;
    }

    const glm::vec3 cp = p - c;
    const float d5 = glm::dot(ab, cp);
    const float d6 = glm::dot(ac, cp);
    if (d6 >= 0.0f && d5 <= d6) return c;

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        const float w = d2 / (d2 - d6);
        return a + w * ac;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + ab * v + ac * w;
}

bool rayIntersectsTriangle(const glm::vec3& o,
                           const glm::vec3& d,
                           const glm::vec3& a,
                           const glm::vec3& b,
                           const glm::vec3& c,
                           float& tOut)
{
    constexpr float EPS = 1e-7f;
    const glm::vec3 e1 = b - a;
    const glm::vec3 e2 = c - a;
    const glm::vec3 p = glm::cross(d, e2);
    const float det = glm::dot(e1, p);
    if (std::fabs(det) < EPS) return false;

    const float invDet = 1.0f / det;
    const glm::vec3 tv = o - a;
    const float u = glm::dot(tv, p) * invDet;
    if (u < -EPS || u > 1.0f + EPS) return false;

    const glm::vec3 q = glm::cross(tv, e1);
    const float v = glm::dot(d, q) * invDet;
    if (v < -EPS || u + v > 1.0f + EPS) return false;

    const float t = glm::dot(e2, q) * invDet;
    if (t <= EPS) return false;
    tOut = t;
    return true;
}

bool pointInsideClosedMesh(const Mesh& mesh, const glm::vec3& p)
{
    const glm::vec3 rayDir =
        glm::normalize(glm::vec3(1.0f, 0.37139067f, 0.17320508f));

    std::vector<float> hits;
    hits.reserve(16);

    for (const Tri& tri : mesh.triangles) {
        float t = 0.0f;
        if (!rayIntersectsTriangle(p, rayDir,
                                   mesh.vertices[tri.a],
                                   mesh.vertices[tri.b],
                                   mesh.vertices[tri.c],
                                   t))
        {
            continue;
        }

        bool duplicate = false;
        for (float oldT : hits) {
            if (std::fabs(oldT - t) < 1e-5f) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) hits.push_back(t);
    }

    return (hits.size() % 2) == 1;
}

bool penetrationAgainstMesh(const Mesh& mesh,
                            const glm::vec3& pLocal,
                            float slop,
                            float& outDepth,
                            glm::vec3& outNormalLocal)
{
    float bestD2 = std::numeric_limits<float>::max();
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);

    for (const Tri& tri : mesh.triangles) {
        const glm::vec3& a = mesh.vertices[tri.a];
        const glm::vec3& b = mesh.vertices[tri.b];
        const glm::vec3& c = mesh.vertices[tri.c];

        const glm::vec3 closest = closestPointOnTriangle(pLocal, a, b, c);
        const glm::vec3 delta = pLocal - closest;
        const float d2 = glm::dot(delta, delta);
        if (d2 >= bestD2) continue;

        glm::vec3 n = glm::cross(b - a, c - a);
        const float nLen = glm::length(n);
        if (nLen <= 1e-8f) continue;
        n /= nLen;

        const glm::vec3 centroid = (a + b + c) / 3.0f;
        if (glm::dot(n, centroid) < 0.0f) n = -n;

        bestD2 = d2;
        bestNormal = n;
    }

    if (bestD2 == std::numeric_limits<float>::max()) return false;

    outDepth = std::sqrt(bestD2);
    if (outDepth <= slop) return false;
    if (!pointInsideClosedMesh(mesh, pLocal)) return false;

    outNormalLocal = bestNormal;
    return true;
}

struct Contact {
    glm::vec3 point{0.0f};
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    float penetration = 0.0f;
};

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

// Body-body collision: confirmed surface contacts per pair.
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

            // Narrow phase: use the other body's local AABB as a cheap
            // reject, then confirm against its closed mesh surface. Keep
            // several contacts so face-on-face uniform blocks cannot pivot
            // around one corner and sink into each other.
            std::vector<Contact> contacts;
            contacts.reserve(16);

            auto probe = [&](RigidBody* P, RigidBody* Q) {
                auto probeLocalPoint = [&](const glm::vec3& pLocal) {
                    glm::vec3 w = P->bodyToWorld(pLocal);
                    glm::vec3 qLocal = Q->worldToBody(w);
                    if (qLocal.x < Q->aabbMin.x || qLocal.x > Q->aabbMax.x) return;
                    if (qLocal.y < Q->aabbMin.y || qLocal.y > Q->aabbMax.y) return;
                    if (qLocal.z < Q->aabbMin.z || qLocal.z > Q->aabbMax.z) return;

                    float pen = 0.0f;
                    glm::vec3 nLocal(0.0f, 1.0f, 0.0f);
                    if (!penetrationAgainstMesh(Q->meshLocal, qLocal,
                                                positionSlop, pen, nLocal))
                    {
                        return;
                    }

                    glm::vec3 n = glm::normalize(Q->orientation * nLocal);
                    if (P == b && Q == a) n = -n;
                    contacts.push_back({ w, n, pen });
                };

                for (const auto& vLocal : P->meshLocal.vertices) {
                    probeLocalPoint(vLocal);
                }

                // Face-on-face uniform blocks can overlap without any corner
                // being strictly inside the other mesh. Triangle centroids
                // give those broad faces contact points of their own.
                for (const Tri& tri : P->meshLocal.triangles) {
                    const glm::vec3 centroid =
                        (P->meshLocal.vertices[tri.a] +
                         P->meshLocal.vertices[tri.b] +
                         P->meshLocal.vertices[tri.c]) / 3.0f;
                    probeLocalPoint(centroid);
                }
            };

            probe(a, b);
            probe(b, a);

            if (!contacts.empty()) {
                std::sort(contacts.begin(), contacts.end(),
                    [](const Contact& lhs, const Contact& rhs) {
                        return lhs.penetration > rhs.penetration;
                    });

                const std::size_t maxContacts = std::min<std::size_t>(
                    contacts.size(), 8);
                for (std::size_t c = 0; c < maxContacts; ++c) {
                    applyVelocityImpulse(a, b, contacts[c].point,
                                         contacts[c].normal,
                                         restitutionThreshold);
                }

                const Contact& deepest = contacts.front();
                applyPositionalCorrection(a, b, deepest.normal,
                                          deepest.penetration,
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
