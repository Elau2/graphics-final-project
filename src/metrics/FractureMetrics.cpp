// FractureMetrics.cpp
#include "metrics/FractureMetrics.h"
#include "physics/PhysicsWorld.h"

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace destruct {

// ---------------------------------------------------------------------------
// Signed-volume formula for a triangle's contribution to mesh volume.
// Summing over all triangles gives the signed volume of the closed mesh.
// Reference: Zhang & Chen, "Efficient Feature Extraction for 2D/3D Objects
// in Mesh Representation"
// ---------------------------------------------------------------------------
static float signedTriVolume(const glm::vec3& a,
                             const glm::vec3& b,
                             const glm::vec3& c)
{
    return glm::dot(a, glm::cross(b, c)) / 6.0f;
}

static float meshVolume(const Mesh& m)
{
    float vol = 0.0f;
    for (const Tri& tri : m.triangles) {
        vol += signedTriVolume(m.vertices[tri.a],
                               m.vertices[tri.b],
                               m.vertices[tri.c]);
    }
    return std::fabs(vol);
}

// ---------------------------------------------------------------------------
// Convexity test: a mesh is convex if every vertex lies on or behind every
// face plane. We use a fast approximate version: for each triangle, check
// that all other vertices have dot(n, v - p) <= eps. 
// ---------------------------------------------------------------------------
static bool isConvex(const Mesh& m, float eps = 1e-3f)
{
    for (const Tri& tri : m.triangles) {
        const glm::vec3& a = m.vertices[tri.a];
        const glm::vec3& b = m.vertices[tri.b];
        const glm::vec3& c = m.vertices[tri.c];
        glm::vec3 n = glm::cross(b - a, c - a);
        float nLen = glm::length(n);
        if (nLen < 1e-8f) continue;
        n /= nLen;

        for (const glm::vec3& v : m.vertices) {
            if (glm::dot(n, v - a) > eps) return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// FragmentStats
// ---------------------------------------------------------------------------
FragmentStats computeFragmentStats(const Mesh& fragment)
{
    FragmentStats s;
    if (fragment.empty()) return s;

    s.volume   = meshVolume(fragment);
    s.isConvex = isConvex(fragment);

    // AABB aspect ratio
    glm::vec3 bbMin, bbMax;
    fragment.computeAABB(bbMin, bbMax);
    glm::vec3 ext = bbMax - bbMin;

    float sortedSides[3] = { ext.x, ext.y, ext.z };
    std::sort(sortedSides, sortedSides + 3);
    // Avoid divide-by-zero for degenerate fragments
    s.aspectRatio = (sortedSides[0] > 1e-6f)
                  ? sortedSides[2] / sortedSides[0]
                  : 999.0f;

    return s;
}

// ---------------------------------------------------------------------------
// Main fracture metric computation
// ---------------------------------------------------------------------------
void computeFractureMetrics(const Mesh& original,
                            const std::vector<Mesh>& fragments,
                            int requested,
                            FractureMetrics& out)
{
    out = FractureMetrics{}; // reset
    out.requestedFragments = requested;
    out.actualFragments    = (int)fragments.size();
    out.yieldRate = (requested > 0)
                  ? (float)out.actualFragments / (float)requested
                  : 0.0f;

    out.originalVolume = meshVolume(original);

    if (fragments.empty()) return;

    // Per-fragment stats
    out.perFragment.resize(fragments.size());
    int convexCount = 0;
    float sumAspect = 0.0f;

    for (std::size_t i = 0; i < fragments.size(); ++i) {
        out.perFragment[i] = computeFragmentStats(fragments[i]);
        out.totalFragmentVolume += out.perFragment[i].volume;
        if (out.perFragment[i].isConvex) ++convexCount;
        sumAspect += out.perFragment[i].aspectRatio;
    }

    // Volume conservation
    out.volumeConservation = (out.originalVolume > 1e-8f)
                           ? out.totalFragmentVolume / out.originalVolume
                           : 0.0f;

    // Volume distribution stats
    std::vector<float> vols;
    vols.reserve(fragments.size());
    for (const auto& fs : out.perFragment) vols.push_back(fs.volume);

    out.minVolume = *std::min_element(vols.begin(), vols.end());
    out.maxVolume = *std::max_element(vols.begin(), vols.end());
    out.meanVolume = out.totalFragmentVolume / (float)fragments.size();

    float variance = 0.0f;
    for (float v : vols) {
        float d = v - out.meanVolume;
        variance += d * d;
    }
    variance /= (float)fragments.size();
    out.stddevVolume = std::sqrt(variance);

    out.uniformityScore = (out.meanVolume > 1e-8f)
                        ? std::max(0.0f, 1.0f - out.stddevVolume / out.meanVolume)
                        : 0.0f;

    // Pattern metrics
    out.convexityRatio  = (float)convexCount / (float)fragments.size();
    out.meanAspectRatio = sumAspect / (float)fragments.size();
}

// ---------------------------------------------------------------------------
// Physics / settling metrics 
// ---------------------------------------------------------------------------
void updatePhysicsMetrics(float dt,
                          const PhysicsWorld& world,
                          FractureMetrics& out)
{
    bool allSleeping = true;
    float totalKE    = 0.0f;
    float totalOverlap = 0.0f; // crude: summed |y_below_ground| 
    
    for (const auto& bPtr : world.bodies) {
        const RigidBody* b = bPtr.get();
        if (b->isStatic()) continue;

        if (!b->sleeping) {
            allSleeping = false;

            float linKE = 0.5f * b->mass *
                          glm::dot(b->linearVelocity, b->linearVelocity);
            // Rotational KE ≈ 0.5 * omega^T * I * omega; approximate with
            // scalar inertia estimate to avoid full matrix multiply here.
            float angKE = 0.5f * glm::dot(b->angularVelocity,
                                           b->angularVelocity);
            totalKE += linKE + angKE;
        }

        // Overlap proxy: how far below ground is the lowest bounding point?
        float lowestY = b->position.y - b->boundingRadius;
        if (lowestY < world.groundY) {
            totalOverlap += world.groundY - lowestY;
        }
    }

    out.peakKineticEnergy = std::max(out.peakKineticEnergy, totalKE);

    if (!allSleeping) {
        out.settlingTime += dt;
    } else {
        // Settling complete — record final overlap error once.
        out.finalOverlapError = totalOverlap;
    }
}

// ---------------------------------------------------------------------------
// Evaluate pass/fail
// ---------------------------------------------------------------------------
FractureMetrics::PassFail
FractureMetrics::evaluate(const Thresholds& t) const
{
    PassFail pf;
    pf.yieldRate          = (yieldRate          >= t.minYieldRate);
    pf.volumeConservation = (volumeConservation  >= t.minVolumeConservation &&
                             volumeConservation  <= t.maxVolumeConservation);
    pf.uniformity         = (uniformityScore     >= t.minUniformityScore);
    pf.convexity          = (convexityRatio      >= t.minConvexityRatio);
    pf.aspectRatio        = (meanAspectRatio     <= t.maxMeanAspectRatio);
    pf.settlingTime       = (settlingTime        <= t.maxSettlingTime);
    pf.overlapError       = (finalOverlapError   <= t.maxFinalOverlapError);
    return pf;
}

// ---------------------------------------------------------------------------
// Summary string
// ---------------------------------------------------------------------------
std::string FractureMetrics::summary() const
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "Fragments: " << actualFragments << "/" << requestedFragments
       << "  Yield: "       << yieldRate          * 100.f << "%"
       << "  VolConserv: "  << volumeConservation * 100.f << "%"
       << "  Uniformity: "  << uniformityScore    * 100.f << "%"
       << "  Convex: "      << convexityRatio     * 100.f << "%"
       << "  AspectRatio: " << meanAspectRatio
       << "  Settling: "    << settlingTime        << "s"
       << "  Overlap: "     << finalOverlapError;
    return ss.str();
}

} // namespace destruct
