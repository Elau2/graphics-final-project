#pragma once
// UniformFracture.h
//
// Alternative fracture method: axis-aligned uniform grid cutting.
//
// This is included as a *comparison benchmark* for the Voronoi method
// (addressing the TA's suggestion of benchmarking against alternative
// approaches). Instead of sampling random seed points and cutting along
// their perpendicular bisectors, we cut the mesh along a regular 3D grid
// of axis-aligned planes. Every fragment is therefore an axis-aligned
// "brick" intersected with the original mesh.
//
// Trade-offs vs Voronoi:
//   + trivially deterministic, trivially uniform size distribution
//   + no seed-placement problem, no chance of empty cells
//   + fragments are axis-aligned, which makes them look "blocky" and
//     unrealistic (this is exactly the qualitative difference we want
//     to highlight in the writeup)
//   - no way to bias fragments toward an impact
//   - does not match the look of brittle-material fracture in reality
//
// Providing both methods with the same public signature makes A/B
// comparison on identical input meshes a one-line change.

#include "mesh/Mesh.h"
#include <vector>

namespace destruct {

struct UniformFractureParams {
    // Number of cells along each axis. Total fragments <= nx*ny*nz
    // (some cells may fall outside the mesh and yield nothing).
    int nx = 3;
    int ny = 3;
    int nz = 3;
};

// Cut `mesh` into axis-aligned brick fragments; append each non-empty
// fragment to `outFragments`. Returns the number produced.
std::size_t
uniformFracture(const Mesh& mesh,
                const UniformFractureParams& params,
                std::vector<Mesh>& outFragments);

} // namespace destruct
