// PhysicsWorld.cpp
#include "physics/PhysicsWorld.h"
 
#include <glm/gtc/quaternion.hpp>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/gtx/quaternion.hpp>
 
#include <algorithm>
#include <array>
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
    cachedPairs.clear();
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
 
        // Single canonical refresh after orientation update.
        // integrateForces already refreshed before its impulse math,
        // so we only need one more here. The old code did this twice.
        b->refreshInertiaWorld();
    }
 
    // 3. Collision detection + impulse resolution.
    collideGround();
 
    // First iteration: full broad-phase detection + resolve, cache pairs.
    collideBodiesDetect();
    collideBodiesResolve();
 
    // Subsequent iterations: re-resolve cached pairs only (skip detection).
    constexpr int kExtraIterations = 3;
    for (int i = 0; i < kExtraIterations; ++i) {
        collideBodiesResolve();
    }
 
    // 4. Put slow-moving bodies to sleep.
    updateSleep(dt);
}
 
void PhysicsWorld::integrateForces(float dt)
{
    const float linDampFactor = std::max(0.0f, 1.0f - linearDamping  * dt);
    const float angDampFactor = std::max(0.0f, 1.0f - angularDamping * dt);
 
    for (auto& b : bodies) {
        if (b->isStatic() || b->sleeping) {
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
 
        // Refresh here so impulse math below sees correct inertia.
        // NOTE: we do NOT refresh again after position integration for
        // sleeping bodies — they are skipped in the integrate loop above.
        b->refreshInertiaWorld();
    }
}
 
// ---------------------------------------------------------------------------

namespace {
 
// ---- Velocity impulse (unchanged logic, no allocation) --------------------
 
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
    if (vn >= 0.0f) return;
 
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
            jt = std::max(-mu * jn, std::min(mu * jn, jt));
 
            glm::vec3 impulseT = jt * t;
            a->applyImpulseAtPoint( impulseT, worldPoint);
            if (b) b->applyImpulseAtPoint(-impulseT, worldPoint);
        }
    }
}
 
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
    static const glm::vec3 rayDir =
        glm::normalize(glm::vec3(1.0f, 0.37139067f, 0.17320508f));
 
    // Small fixed dedup buffer on the stack — no heap involvement.
    std::array<float, 16> hitTs;
    int hitCount = 0;
 
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
        for (int k = 0; k < hitCount; ++k) {
            if (std::fabs(hitTs[k] - t) < 1e-5f) { duplicate = true; break; }
        }
        if (!duplicate) {
            if (hitCount < (int)hitTs.size()) hitTs[hitCount++] = t;
            // If we somehow overflow the buffer, treat as even (outside).
            // This is safe for convex meshes and extremely unlikely otherwise.
        }
    }
 
    return (hitCount % 2) == 1;
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
 

 
constexpr int kMaxContacts = 64;
 
} // anonymous namespace
 
// ---------------------------------------------------------------------------
// Ground collision (unchanged logic)
// ---------------------------------------------------------------------------
void PhysicsWorld::collideGround()
{
    const glm::vec3 n(0.0f, 1.0f, 0.0f);
 
    for (auto& bPtr : bodies) {
        RigidBody* b = bPtr.get();
        if (b->isStatic()) continue;
        if (b->position.y - b->boundingRadius > groundY) continue;
 
        float     maxPen = 0.0f;
        glm::vec3 deepest{0.0f};
        bool      hasDeep = false;
 
        for (const auto& vLocal : b->meshLocal.vertices) {
            const glm::vec3 w = b->bodyToWorld(vLocal);
            const float pen = groundY - w.y;
            if (pen > 0.0f) {
                applyVelocityImpulse(b, nullptr, w, n, restitutionThreshold);
                if (pen > maxPen) { maxPen = pen; deepest = w; hasDeep = true; }
            }
        }
 
        if (hasDeep) {
            applyPositionalCorrection(b, nullptr, n, maxPen,
                                      positionSlop, positionPercent);
        }
    }
}
 
// ---------------------------------------------------------------------------
// Body-body: detect (with spatial hash broad phase) + cache pairs
// ---------------------------------------------------------------------------
 
// Spatial hash helpers
static uint64_t spatialHashKey(int x, int y, int z)
{
    // Large primes for good distribution
    return  (uint64_t)(uint32_t)(x * 73856093)
          ^ (uint64_t)(uint32_t)(y * 19349663) << 20
          ^ (uint64_t)(uint32_t)(z * 83492791) << 40;
}
 
void PhysicsWorld::collideBodiesDetect()
{
    cachedPairs.clear();
 
    const std::size_t N = bodies.size();
    if (N < 2) return;
 
    // ---- Spatial hash broad phase ----------------------------------------
    // Cell size = 2 * average bounding radius, clamped to a sensible range.
    // One pass to compute cell size, one to insert, one to query.
 
    float avgR = 0.0f;
    int   activeCount = 0;
    for (auto& b : bodies) {
        if (!b->isStatic()) { avgR += b->boundingRadius; ++activeCount; }
    }
    const float cellSize = (activeCount > 0)
        ? std::max(0.1f, 2.0f * avgR / (float)activeCount)
        : 1.0f;
    const float invCell = 1.0f / cellSize;
 
    // Map cell key -> list of body indices. We use a flat parallel array
    // (keys + values) sorted by key to avoid unordered_map overhead.
    // For small N a sorted vector is cache-friendlier than a hash map.
    struct CellEntry { uint64_t key; uint32_t bodyIdx; };
    std::vector<CellEntry> cellEntries;
    cellEntries.reserve(N * 8); // each body touches ~8 cells
 
    auto cellOf = [&](float v) { return (int)std::floor(v * invCell); };
 
    for (std::size_t i = 0; i < N; ++i) {
        RigidBody* b = bodies[i].get();
        // Static-only bodies still need to receive contacts from dynamic ones,
        // but we only *insert* dynamic bodies as initiators.
        int x0 = cellOf(b->position.x - b->boundingRadius);
        int x1 = cellOf(b->position.x + b->boundingRadius);
        int y0 = cellOf(b->position.y - b->boundingRadius);
        int y1 = cellOf(b->position.y + b->boundingRadius);
        int z0 = cellOf(b->position.z - b->boundingRadius);
        int z1 = cellOf(b->position.z + b->boundingRadius);
        for (int cx = x0; cx <= x1; ++cx)
            for (int cy = y0; cy <= y1; ++cy)
                for (int cz = z0; cz <= z1; ++cz)
                    cellEntries.push_back({ spatialHashKey(cx, cy, cz), (uint32_t)i });
    }
 
    std::sort(cellEntries.begin(), cellEntries.end(),
              [](const CellEntry& a, const CellEntry& b){ return a.key < b.key; });
 
    // Collect candidate pairs: bodies sharing a cell.
    // Use a small set (sorted pair of indices) to deduplicate.
    struct BodyPair { uint32_t a, b; };
    std::vector<BodyPair> candidates;
    candidates.reserve(N * 4);
 
    for (std::size_t s = 0; s < cellEntries.size(); ) {
        uint64_t key = cellEntries[s].key;
        std::size_t e = s;
        while (e < cellEntries.size() && cellEntries[e].key == key) ++e;
        // All bodies in [s,e) share this cell.
        for (std::size_t p = s; p < e; ++p)
            for (std::size_t q = p + 1; q < e; ++q) {
                uint32_t ia = cellEntries[p].bodyIdx;
                uint32_t ib = cellEntries[q].bodyIdx;
                if (ia > ib) std::swap(ia, ib);
                candidates.push_back({ ia, ib });
            }
        s = e;
    }
 
    // Sort + unique to eliminate duplicates from multi-cell overlaps.
    std::sort(candidates.begin(), candidates.end(),
              [](const BodyPair& x, const BodyPair& y){
                  return x.a < y.a || (x.a == y.a && x.b < y.b); });
    candidates.erase(std::unique(candidates.begin(), candidates.end(),
                                 [](const BodyPair& x, const BodyPair& y){
                                     return x.a == y.a && x.b == y.b; }),
                     candidates.end());
 
    // ---- Narrow phase for each candidate pair ----------------------------
 
    for (const auto& cp : candidates) {
        RigidBody* a = bodies[cp.a].get();
        RigidBody* b = bodies[cp.b].get();
 
        if (a->isStatic() && b->isStatic()) continue;
        if (a->sleeping  && b->sleeping)  continue;
 
        // Bounding-sphere reject (the spatial hash is approximate).
        glm::vec3 delta = a->position - b->position;
        float rSum = a->boundingRadius + b->boundingRadius;
        if (glm::dot(delta, delta) > rSum * rSum) continue;
 
        // Wake sleeper if the other body is live and close.
        if (a->sleeping) { a->sleeping = false; a->sleepTimer = 0.0f; }
        if (b->sleeping) { b->sleeping = false; b->sleepTimer = 0.0f; }
 
        // Stack-allocated contact buffer — no heap allocation per pair.
        std::array<Contact, kMaxContacts> contacts;
        int contactCount = 0;
 
        auto probe = [&](RigidBody* P, RigidBody* Q) {
            auto probeLocalPoint = [&](const glm::vec3& pLocal) {
                if (contactCount >= kMaxContacts) return;
 
                glm::vec3 w      = P->bodyToWorld(pLocal);
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
                contacts[contactCount++] = { w, n, pen };
            };
 
            for (const auto& vLocal : P->meshLocal.vertices)
                probeLocalPoint(vLocal);
 
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
 
        if (contactCount == 0) continue;
 
        // Sort by penetration depth (deepest first), then trim to 8.
        std::sort(contacts.begin(), contacts.begin() + contactCount,
            [](const Contact& lhs, const Contact& rhs) {
                return lhs.penetration > rhs.penetration;
            });
        const int resolveCount = std::min(contactCount, 8);
 
        // Cache the pair for subsequent re-resolve iterations.
        CachedPair pair;
        pair.a = a;
        pair.b = b;
        pair.contactCount = resolveCount;
        for (int k = 0; k < resolveCount; ++k)
            pair.contacts[k] = contacts[k];
        cachedPairs.emplace_back(std::move(pair));
    }
}
 
// Re-resolve cached pairs without repeating detection.
void PhysicsWorld::collideBodiesResolve()
{
    for (auto& pair : cachedPairs) {
        RigidBody* a = pair.a;
        RigidBody* b = pair.b;
 
        for (int c = 0; c < pair.contactCount; ++c) {
            applyVelocityImpulse(a, b,
                                 pair.contacts[c].point,
                                 pair.contacts[c].normal,
                                 restitutionThreshold);
        }
 
        const auto& deepest = pair.contacts[0];
        applyPositionalCorrection(a, b,
                                  deepest.normal, deepest.penetration,
                                  positionSlop, positionPercent);
    }
}
 
// ---------------------------------------------------------------------------
// Sleep latching (unchanged)
// ---------------------------------------------------------------------------
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