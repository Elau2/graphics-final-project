#pragma once
// VoronoiFracture.h
//
// Fracture a closed mesh into Voronoi-shaped fragments.
//
// Given a set of seed points inside the mesh volume, each fragment is the
// intersection of the original mesh with the Voronoi cell of its seed.
// The Voronoi cell of seed i is the region { x : |x - s_i| <= |x - s_j|
// for all j != i }, which is the intersection of half-spaces defined by
// the perpendicular bisector planes between s_i and every other seed s_j.
//
// We do NOT build a full 3D Voronoi diagram (too fiddly, requires convex
// hulls and incremental construction). Instead we exploit the fact that
// MeshClipper can cut any closed mesh by a plane, and that intersection
// is associative: iteratively clipping by all bisector planes yields the
// Voronoi cell directly. This is O(N^2) in the number of seeds, which
// is fine for N in the tens to low hundreds (our target range).
//
// References:
//   Müller et al., "Real Time Dynamic Fracture", SIGGRAPH 2013.
//   https://www.joesfer.com/?p=60  (iterative-clip Voronoi write-up)

#include "mesh/Mesh.h"
#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace destruct {

struct VoronoiParams {
    // How many seeds (= fragments) to generate when using the random
    // sampling helpers.
    int numSeeds = 16;

    // RNG seed for reproducibility. 0 => pick something time-based.
    uint32_t rngSeed = 1u;

    // If > 0, use Poisson-disk-style rejection to keep seeds at least this
    // far apart (in world units). Useful for more even fragment sizes.
    // Set to 0 to disable.
    float minSeedSeparation = 0.0f;

    // Impact-biased seeding (aspirational feature from the proposal).
    // If useImpactBias is true, seeds are drawn from a Gaussian around
    // impactPoint with stddev impactRadius instead of uniformly in the AABB.
    // This concentrates fragments near the impact, producing more realistic
    // "crater"-style destruction.
    bool      useImpactBias = false;
    glm::vec3 impactPoint{0.0f};
    float     impactRadius = 0.25f;
};

// Generate seed points inside the AABB of `mesh`. When impact bias is on,
// seeds cluster around params.impactPoint. When Poisson-disk separation is
// set, extra rejected samples are drawn until numSeeds survive (bounded
// attempts so the function always terminates).
std::vector<glm::vec3>
generateSeeds(const Mesh& mesh, const VoronoiParams& params);

// Run the fracture: clip `mesh` once per seed and append each non-empty
// fragment to `outFragments`. Vertex normals are recomputed for every
// fragment so lighting works immediately.
//
// Returns the number of fragments produced (may be < seeds.size() if some
// seeds yielded empty cells, e.g. outside the mesh).
std::size_t
voronoiFracture(const Mesh& mesh,
                const std::vector<glm::vec3>& seeds,
                std::vector<Mesh>& outFragments);

// Convenience: generate seeds with params, then fracture. Returns the seeds
// actually used so the caller can visualise them if desired.
std::vector<glm::vec3>
voronoiFracture(const Mesh& mesh,
                const VoronoiParams& params,
                std::vector<Mesh>& outFragments);

} // namespace destruct
