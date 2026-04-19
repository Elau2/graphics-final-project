#pragma once
// MeshClipper.h
//
// Slice a closed triangle mesh by a plane and keep the "inside" half
// (the side with negative signed distance, i.e. dot(n, x) < d).
//
// This is essentially Sutherland-Hodgman extended to 3D triangle meshes,
// plus a cap-triangulation step that closes the exposed cross-section
// so that each fragment remains watertight. Watertightness matters
// because the mass-property integrals assume a closed surface.
//
// Note: the cap triangulation uses a fan around the centroid of the cut
// polygon. That is optimal for convex caps (always correct) and works
// well in practice for Voronoi cells of a convex hull. For strongly
// non-convex cuts it can produce visual artefacts but never leaves the
// mesh open, because every cut edge is still used as the outer edge of
// some cap triangle.

#include "mesh/Mesh.h"
#include "math/Plane.h"

namespace destruct {

class MeshClipper {
public:
    // Clip `input` by `plane`, writing the resulting watertight fragment
    // to `output`. `output` is cleared first. Returns true if the result
    // is non-empty (i.e. something of the mesh survived the clip).
    //
    // If the whole mesh is on the inside of the plane, `output` is an
    // exact copy of `input` (no cap generated, nothing to close).
    static bool clip(const Mesh& input, const Plane& plane, Mesh& output);

    // Convenience: clip `input` by every plane in `planes` in sequence.
    // Each clip may shrink the mesh; the routine bails out early if the
    // mesh becomes empty.
    static bool clipAgainstAll(const Mesh& input,
                               const std::vector<Plane>& planes,
                               Mesh& output);
};

} // namespace destruct
