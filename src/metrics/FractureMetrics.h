#pragma once
// FractureMetrics.h
//
// Quantitative success metrics for the destruction simulator, addressing
// the TA feedback: "clearly define what constitutes a successful vs failed
// fracture."
//
// Three categories of metrics are computed:
//
//  1. FRAGMENT metrics  (computed immediately after fracture)
//     - Fragment count vs requested seed count
//     - Volume conservation: sum(fragment volumes) / original volume
//     - Size distribution: mean, stddev, min, max of fragment volumes
//     - Uniformity score: 1 - (stddev / mean), in [0,1]. 1 = perfectly equal
//       fragments; 0 = one fragment dominates.
//
//  2. PATTERN metrics  (computed after fracture, geometry-based)
//     - Convexity ratio: fraction of fragments that are convex
//       (a Voronoi cell is always convex; <1.0 indicates clipping artefacts)
//     - Average aspect ratio of each fragment's AABB (1 = cube, >1 = slab/needle)
//
//  3. PHYSICS / SETTLING metrics  (computed each step from PhysicsWorld)
//     - Settling time: seconds until all bodies sleep
//     - Peak KE: maximum total kinetic energy seen during settling
//     - Final overlap error: sum of penetration depths after settling
//       (should be ~0; large values mean the physics failed to resolve)
//
// SUCCESS / FAILURE thresholds are stored in FractureMetrics::Thresholds
// and compared in FractureMetrics::evaluate(). Each metric returns a
// PassFail status so the overlay can colour-code green/red.

#include "mesh/Mesh.h"

#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace destruct {

// Forward declare so we don't pull in the whole physics header here.
class PhysicsWorld;

// ---------------------------------------------------------------------------
// Per-fragment geometry stats
// ---------------------------------------------------------------------------
struct FragmentStats {
    float volume      = 0.0f;
    float aspectRatio = 0.0f; // longest AABB side / shortest AABB side
    bool  isConvex    = false;
};

// ---------------------------------------------------------------------------
// All metrics in one flat struct — easy to log or display
// ---------------------------------------------------------------------------
struct FractureMetrics {

    // --- Fragment metrics ---
    int   requestedFragments = 0; // seeds requested
    int   actualFragments    = 0; // non-empty cells produced
    float yieldRate          = 0.0f; // actualFragments / requestedFragments

    float originalVolume     = 0.0f;
    float totalFragmentVolume= 0.0f;
    float volumeConservation = 0.0f; // totalFragment / original, ideal = 1

    float meanVolume   = 0.0f;
    float stddevVolume = 0.0f;
    float minVolume    = 0.0f;
    float maxVolume    = 0.0f;
    float uniformityScore = 0.0f; // 1 - stddev/mean

    // --- Pattern metrics ---
    float convexityRatio = 0.0f; // fraction of convex fragments
    float meanAspectRatio= 0.0f; // 1 = cubes, higher = slabs

    // --- Physics / settling metrics ---
    float settlingTime     = 0.0f; // seconds until all bodies asleep
    float peakKineticEnergy= 0.0f;
    float finalOverlapError= 0.0f; // total penetration depth at rest

    // --- Per-fragment detail (optional, for histogram) ---
    std::vector<FragmentStats> perFragment;

    // -----------------------------------------------------------------------
    // Thresholds that define pass / fail
    // -----------------------------------------------------------------------
    struct Thresholds {
        float minYieldRate        = 0.80f; // at least 80% of seeds produce fragments
        float minVolumeConservation = 0.95f; // lose at most 5% of volume
        float maxVolumeConservation = 1.02f; // gained volume = overlap artefact
        float minUniformityScore  = 0.40f; // Voronoi should be reasonably even
        float minConvexityRatio   = 0.90f; // >90% of cells must be convex
        float maxMeanAspectRatio  = 4.0f;  // no needle-like fragments on average
        float maxSettlingTime     = 5.0f;  // seconds
        float maxFinalOverlapError= 0.01f; // metres total penetration at rest
    };

    // -----------------------------------------------------------------------
    // Pass/fail result per metric
    // -----------------------------------------------------------------------
    struct PassFail {
        bool yieldRate         = false;
        bool volumeConservation= false;
        bool uniformity        = false;
        bool convexity         = false;
        bool aspectRatio       = false;
        bool settlingTime      = false;
        bool overlapError      = false;

        bool allPass() const {
            return yieldRate && volumeConservation && uniformity
                && convexity && aspectRatio && settlingTime && overlapError;
        }
    };

    // Evaluate all metrics against thresholds. Call after compute*().
    // Pass a Thresholds{} for defaults.
    PassFail evaluate(const Thresholds& t) const;
    PassFail evaluate() const; // uses Thresholds{}

    // Human-readable one-liner summary, e.g. for console logging.
    std::string summary() const;
};

// Convenience: evaluate against default thresholds (no args needed).
inline FractureMetrics::PassFail FractureMetrics::evaluate() const
{
    return evaluate(Thresholds{});
}

// ---------------------------------------------------------------------------
// Computation functions
// ---------------------------------------------------------------------------

// Compute volume and aspect ratio of a single fragment mesh.
FragmentStats computeFragmentStats(const Mesh& fragment);

// Populate all fragment + pattern metrics. Call right after fracture.
// `original`   = the mesh before fracture.
// `fragments`  = output of voronoiFracture / uniformFracture.
// `requested`  = params.numSeeds (or nx*ny*nz for uniform).
void computeFractureMetrics(const Mesh& original,
                            const std::vector<Mesh>& fragments,
                            int requested,
                            FractureMetrics& out);

// Call once per physics step to track settling. Must be called on the
// same FractureMetrics object until all bodies are asleep.
// `dt`    = step delta time.
// `world` = the physics world (reads body velocities and positions).
void updatePhysicsMetrics(float dt,
                          const PhysicsWorld& world,
                          FractureMetrics& out);

} // namespace destruct
