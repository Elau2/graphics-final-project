#pragma once
// MeshProperties.h
//
// Compute volume, center of mass, and inertia tensor of a closed triangle
// mesh by summing signed tetrahedra (triangle + origin). This is the
// standard divergence-theorem-based trick: any volume integral becomes a
// sum of per-tetrahedron integrals, each of which has a closed-form value
// that depends only on the tetrahedron's four vertex positions.
//
// References:
//   Mirtich, "Fast and Accurate Computation of Polyhedral Mass Properties",
//   Journal of Graphics Tools, 1996.
//
// The results are returned for density = 1. Callers scale by the chosen
// density afterward (see RigidBody).

#include "mesh/Mesh.h"
#include <glm/glm.hpp>
#include <glm/mat3x3.hpp>

namespace destruct {

struct MassProperties {
    float     volume = 0.0f;           // > 0 for correctly oriented closed mesh
    glm::vec3 centerOfMass{0.0f};      // world-frame coordinates
    glm::mat3 inertiaAboutCoM{1.0f};   // 3x3 inertia tensor about CoM, density = 1
};

// Compute mass properties of `mesh`. Returns MassProperties with zero volume
// if the mesh is empty / not closed / oriented inward.
MassProperties computeMassProperties(const Mesh& mesh);

} // namespace destruct
