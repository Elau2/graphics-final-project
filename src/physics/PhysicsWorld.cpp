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
    collideGround(dt);
 
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

// Forward declaration — defined below alongside closestPointOnTriangle.
static bool rayIntersectsTriangle(const glm::vec3& o, const glm::vec3& d,
                                   const glm::vec3& a, const glm::vec3& b,
                                   const glm::vec3& c, float& tOut);

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

// Squared distance from point p to an AABB.
static float distSqToAABB(const glm::vec3& p,
                           const glm::vec3& mn, const glm::vec3& mx)
{
    float dx = std::max(0.0f, std::max(mn.x - p.x, p.x - mx.x));
    float dy = std::max(0.0f, std::max(mn.y - p.y, p.y - mx.y));
    float dz = std::max(0.0f, std::max(mn.z - p.z, p.z - mx.z));
    return dx*dx + dy*dy + dz*dz;
}

// Slab test: does the infinite ray (origin o, precomputed invDir 1/d) hit AABB?
static bool rayHitsAABB(const glm::vec3& o, const glm::vec3& invD,
                         const glm::vec3& mn, const glm::vec3& mx)
{
    float tmin = -std::numeric_limits<float>::max();
    float tmax =  std::numeric_limits<float>::max();
    for (int i = 0; i < 3; ++i) {
        float t1 = (mn[i] - o[i]) * invD[i];
        float t2 = (mx[i] - o[i]) * invD[i];
        if (t1 > t2) std::swap(t1, t2);
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
    }
    return tmax >= 0.0f && tmin <= tmax;
}

// ---------------------------------------------------------------------------
// BVH traversals
// ---------------------------------------------------------------------------

// Branch-and-bound closest-point search. Updates bestD2 / bestNormal in place.
static void bvhFindClosest(const std::vector<BVHNode>& bvh,
                            const std::vector<glm::vec3>& verts,
                            const std::vector<Tri>& tris,
                            int nodeIdx,
                            const glm::vec3& p,
                            float& bestD2,
                            glm::vec3& bestNormal)
{
    const BVHNode& node = bvh[nodeIdx];
    if (distSqToAABB(p, node.aabbMin, node.aabbMax) >= bestD2) return;

    if (node.left == -1) {  // leaf
        const Tri& tri = tris[node.triIdx];
        const glm::vec3& a = verts[tri.a];
        const glm::vec3& b = verts[tri.b];
        const glm::vec3& c = verts[tri.c];

        // Closest point on triangle (Ericson's method, same as closestPointOnTriangle).
        const glm::vec3 ab = b - a, ac = c - a, ap = p - a;
        const float d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
        glm::vec3 closest;
        if (d1 <= 0.0f && d2 <= 0.0f) { closest = a; }
        else {
            const glm::vec3 bp = p - b;
            const float d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) { closest = b; }
            else {
                const float vc = d1*d4 - d3*d2;
                if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
                    closest = a + (d1 / (d1 - d3)) * ab;
                } else {
                    const glm::vec3 cp = p - c;
                    const float d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
                    if (d6 >= 0.0f && d5 <= d6) { closest = c; }
                    else {
                        const float vb = d5*d2 - d1*d6;
                        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
                            closest = a + (d2 / (d2 - d6)) * ac;
                        } else {
                            const float va = d3*d6 - d5*d4;
                            if (va <= 0.0f && (d4-d3) >= 0.0f && (d5-d6) >= 0.0f) {
                                closest = b + ((d4-d3)/((d4-d3)+(d5-d6))) * (c - b);
                            } else {
                                const float denom = 1.0f / (va + vb + vc);
                                closest = a + ab*(vb*denom) + ac*(vc*denom);
                            }
                        }
                    }
                }
            }
        }

        const float d2val = glm::dot(p - closest, p - closest);
        if (d2val < bestD2) {
            glm::vec3 n = glm::cross(b - a, c - a);
            const float nLen = glm::length(n);
            if (nLen > 1e-8f) {
                n /= nLen;
                if (glm::dot(n, (a + b + c) / 3.0f) < 0.0f) n = -n;
                bestD2 = d2val;
                bestNormal = n;
            }
        }
        return;
    }

    // Visit closer child first for better pruning.
    float dL = distSqToAABB(p, bvh[node.left ].aabbMin, bvh[node.left ].aabbMax);
    float dR = distSqToAABB(p, bvh[node.right].aabbMin, bvh[node.right].aabbMax);
    if (dL <= dR) {
        bvhFindClosest(bvh, verts, tris, node.left,  p, bestD2, bestNormal);
        bvhFindClosest(bvh, verts, tris, node.right, p, bestD2, bestNormal);
    } else {
        bvhFindClosest(bvh, verts, tris, node.right, p, bestD2, bestNormal);
        bvhFindClosest(bvh, verts, tris, node.left,  p, bestD2, bestNormal);
    }
}

// Count forward ray hits for the odd-even inside test, deduplicating shared edges.
static void bvhRayCast(const std::vector<BVHNode>& bvh,
                        const std::vector<glm::vec3>& verts,
                        const std::vector<Tri>& tris,
                        int nodeIdx,
                        const glm::vec3& o,
                        const glm::vec3& d,
                        const glm::vec3& invD,
                        std::array<float, 16>& hitTs,
                        int& hitCount)
{
    const BVHNode& node = bvh[nodeIdx];
    if (!rayHitsAABB(o, invD, node.aabbMin, node.aabbMax)) return;

    if (node.left == -1) {  // leaf
        const Tri& tri = tris[node.triIdx];
        float t = 0.0f;
        if (!rayIntersectsTriangle(o, d,
                                   verts[tri.a], verts[tri.b], verts[tri.c], t))
            return;
        for (int k = 0; k < hitCount; ++k)
            if (std::fabs(hitTs[k] - t) < 1e-5f) return;  // duplicate
        if (hitCount < (int)hitTs.size()) hitTs[hitCount++] = t;
        return;
    }

    bvhRayCast(bvh, verts, tris, node.left,  o, d, invD, hitTs, hitCount);
    bvhRayCast(bvh, verts, tris, node.right, o, d, invD, hitTs, hitCount);
}

// ---------------------------------------------------------------------------
// Impulse / correction helpers
// ---------------------------------------------------------------------------

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

// Unified closest-point + inside test. Uses BVH when available (O(log T)),
// falls back to a single combined linear pass over triangles.
bool penetrationAgainstMesh(const Mesh& mesh,
                            const glm::vec3& pLocal,
                            float slop,
                            float& outDepth,
                            glm::vec3& outNormalLocal)
{
    static const glm::vec3 kRayDir =
        glm::normalize(glm::vec3(1.0f, 0.37139067f, 0.17320508f));

    float bestD2 = std::numeric_limits<float>::max();
    glm::vec3 bestNormal(0.0f, 1.0f, 0.0f);
    std::array<float, 16> hitTs;
    int hitCount = 0;

    if (!mesh.bvh.empty()) {
        bvhFindClosest(mesh.bvh, mesh.vertices, mesh.triangles, 0,
                       pLocal, bestD2, bestNormal);
        // Early-out: if the probe is within slop distance of the surface it can't
        // generate a meaningful contact — skip the ray-cast entirely.
        if (bestD2 == std::numeric_limits<float>::max()) return false;
        outDepth = std::sqrt(bestD2);
        if (outDepth <= slop) return false;
        const glm::vec3 invD(1.0f / kRayDir.x, 1.0f / kRayDir.y, 1.0f / kRayDir.z);
        bvhRayCast(mesh.bvh, mesh.vertices, mesh.triangles, 0,
                   pLocal, kRayDir, invD, hitTs, hitCount);
        if ((hitCount % 2) == 0) return false;
        outNormalLocal = bestNormal;
        return true;
    } else {
        // Single combined linear pass: closest-point + ray-cast together.
        for (const Tri& tri : mesh.triangles) {
            const glm::vec3& a = mesh.vertices[tri.a];
            const glm::vec3& b = mesh.vertices[tri.b];
            const glm::vec3& c = mesh.vertices[tri.c];

            const glm::vec3 closest = closestPointOnTriangle(pLocal, a, b, c);
            const float d2 = glm::dot(pLocal - closest, pLocal - closest);
            if (d2 < bestD2) {
                glm::vec3 n = glm::cross(b - a, c - a);
                const float nLen = glm::length(n);
                if (nLen > 1e-8f) {
                    n /= nLen;
                    if (glm::dot(n, (a + b + c) / 3.0f) < 0.0f) n = -n;
                    bestD2 = d2;
                    bestNormal = n;
                }
            }

            float t = 0.0f;
            if (rayIntersectsTriangle(pLocal, kRayDir, a, b, c, t)) {
                bool dup = false;
                for (int k = 0; k < hitCount; ++k)
                    if (std::fabs(hitTs[k] - t) < 1e-5f) { dup = true; break; }
                if (!dup && hitCount < (int)hitTs.size()) hitTs[hitCount++] = t;
            }
        }
    }

    if (bestD2 == std::numeric_limits<float>::max()) return false;
    outDepth = std::sqrt(bestD2);
    if (outDepth <= slop) return false;
    if ((hitCount % 2) == 0) return false;  // odd-even inside test

    outNormalLocal = bestNormal;
    return true;
}
 

 
constexpr int kMaxContacts = 64;
 
} // anonymous namespace

void PhysicsWorld::collideGround(float dt)
{
    const glm::vec3 n(0.0f, 1.0f, 0.0f);

    for (auto& bPtr : bodies) {
        RigidBody* b = bPtr.get();
        if (b->isStatic()) continue;
        if (b->position.y - b->boundingRadius > groundY) continue;

        float     maxPen      = 0.0f;
        glm::vec3 contactSum  {0.0f};
        int       contactCount = 0;

        for (const auto& vLocal : b->meshLocal.vertices) {
            const glm::vec3 w = b->bodyToWorld(vLocal);
            const float pen = groundY - w.y;
            if (pen > 0.0f) {
                contactSum += w;
                ++contactCount;
                if (pen > maxPen) maxPen = pen;
            }
        }

        if (contactCount > 0) {
            // Apply the velocity impulse at the CoM (b->position). ra = 0 means
            // zero cross-product → zero torque → zero angular velocity generated.
            // Any off-centre contact point injects angular velocity every step,
            // which is the root cause of the earthquake rocking.
            applyVelocityImpulse(b, nullptr, b->position, n, restitutionThreshold);
            applyPositionalCorrection(b, nullptr, n, maxPen,
                                      positionSlop, positionPercent);
            // Decay spin so fragments that landed tumbling still come to rest.
            const float angDecay = std::max(0.0f, 1.0f - 12.0f * dt);
            b->angularVelocity *= angDecay;
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
    // Rotate last frame's pairs aside so we can skip narrow-phase for pairs
    // that were zero-contact and still have near-zero relative velocity.
    std::vector<CachedPair> prevPairs;
    prevPairs.swap(cachedPairs);  // cachedPairs now empty; prevPairs has last frame
    std::sort(prevPairs.begin(), prevPairs.end(),
        [](const CachedPair& x, const CachedPair& y){
            if (x.a != y.a) return x.a < y.a;
            return x.b < y.b;
        });

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

        // Zero-relative-motion skip: during free fall all fragments move
        // identically under gravity → relative velocity ≈ 0 and contacts
        // are guaranteed zero. If the previous frame confirmed no contact for
        // this pair AND relative velocity is still tiny, skip the expensive
        // narrow-phase and carry forward the zero-contact entry.
        {
            constexpr float kSkipEps2 = 0.01f * 0.01f; // 1 cm/s threshold
            glm::vec3 rv = a->linearVelocity  - b->linearVelocity;
            glm::vec3 ra = a->angularVelocity - b->angularVelocity;
            if (glm::dot(rv, rv) + glm::dot(ra, ra) < kSkipEps2) {
                CachedPair key{a, b, 0, {}};
                auto it = std::lower_bound(prevPairs.begin(), prevPairs.end(), key,
                    [](const CachedPair& x, const CachedPair& y){
                        if (x.a != y.a) return x.a < y.a;
                        return x.b < y.b;
                    });
                if (it != prevPairs.end() && it->a == a && it->b == b
                    && it->contactCount == 0)
                {
                    cachedPairs.push_back(key); // carry forward for next frame
                    continue;
                }
            }
        }

        // Stack-allocated contact buffer — no heap allocation per pair.
        std::array<Contact, kMaxContacts> contacts;
        int contactCount = 0;
 
        auto probe = [&](RigidBody* P, RigidBody* Q) {
            // Precompute the P-local → Q-local transform once per probe direction.
            // qLocal = PtoQ_rot * pLocal + PtoQ_off
            // This replaces two quaternion*vector ops per probe with one mat3*vec3.
            const glm::mat3 PtoQ_rot =
                glm::mat3_cast(glm::conjugate(Q->orientation) * P->orientation);
            const glm::vec3 PtoQ_off = Q->worldToBody(P->position);

            auto probeLocalPoint = [&](const glm::vec3& pLocal) {
                if (contactCount >= kMaxContacts) return;

                const glm::vec3 qLocal = PtoQ_rot * pLocal + PtoQ_off;
                if (qLocal.x < Q->aabbMin.x || qLocal.x > Q->aabbMax.x) return;
                if (qLocal.y < Q->aabbMin.y || qLocal.y > Q->aabbMax.y) return;
                if (qLocal.z < Q->aabbMin.z || qLocal.z > Q->aabbMax.z) return;

                float pen = 0.0f;
                glm::vec3 nLocal(0.0f, 1.0f, 0.0f);
                if (!penetrationAgainstMesh(Q->meshLocal, qLocal,
                                            positionSlop, pen, nLocal))
                    return;

                // Deferred: only compute world position when we have a real contact.
                const glm::vec3 w = P->bodyToWorld(pLocal);
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
 
        if (contactCount == 0) {
            // Cache zero-contact result so the next frame can skip this pair
            // when relative velocity is still near zero.
            cachedPairs.push_back({a, b, 0, {}});
            continue;
        }

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
        if (pair.contactCount == 0) continue; // zero-contact skip entries

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
            // Drain sub-threshold velocities so bodies converge to rest
            // instead of jittering indefinitely near the sleep boundary.
            const float drain = std::max(0.0f, 1.0f - 8.0f * dt);
            b->linearVelocity  *= drain;
            b->angularVelocity *= drain;
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