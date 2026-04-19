// UniformFracture.cpp
#include "fracture/UniformFracture.h"
#include "mesh/MeshClipper.h"
#include "math/Plane.h"

#include <algorithm>

namespace destruct {

std::size_t
uniformFracture(const Mesh& mesh,
                const UniformFractureParams& params,
                std::vector<Mesh>& outFragments)
{
    if (mesh.empty()) return 0;
    const int nx = std::max(1, params.nx);
    const int ny = std::max(1, params.ny);
    const int nz = std::max(1, params.nz);

    glm::vec3 bbMin, bbMax;
    mesh.computeAABB(bbMin, bbMax);

    // Slightly inflate the box so edges of the mesh are strictly inside
    // the [0, n] grid range; keeps boundary cells from being empty.
    const glm::vec3 ext = bbMax - bbMin;
    bbMin -= 0.001f * ext;
    bbMax += 0.001f * ext;

    const glm::vec3 cell = (bbMax - bbMin) / glm::vec3(nx, ny, nz);

    const std::size_t before = outFragments.size();

    // Six planes per cell. Convention: Plane::fromPointNormal gives inside =
    // dot(n, x) <= d, and MeshClipper keeps the inside. For a cell spanning
    // [x0, x1] on the x axis, the "inside" slab along +x is x <= x1, which
    // is a plane with normal (+1,0,0) and d = x1. The "inside" slab along
    // -x is x >= x0, i.e. -x <= -x0, plane with normal (-1,0,0) and d = -x0.
    std::vector<Plane> planes;
    planes.reserve(6);

    for (int iz = 0; iz < nz; ++iz)
    for (int iy = 0; iy < ny; ++iy)
    for (int ix = 0; ix < nx; ++ix)
    {
        const glm::vec3 lo = bbMin + cell * glm::vec3(ix,     iy,     iz);
        const glm::vec3 hi = bbMin + cell * glm::vec3(ix + 1, iy + 1, iz + 1);

        planes.clear();
        planes.emplace_back(Plane::fromPointNormal(hi, glm::vec3( 1, 0, 0)));
        planes.emplace_back(Plane::fromPointNormal(lo, glm::vec3(-1, 0, 0)));
        planes.emplace_back(Plane::fromPointNormal(hi, glm::vec3( 0, 1, 0)));
        planes.emplace_back(Plane::fromPointNormal(lo, glm::vec3( 0,-1, 0)));
        planes.emplace_back(Plane::fromPointNormal(hi, glm::vec3( 0, 0, 1)));
        planes.emplace_back(Plane::fromPointNormal(lo, glm::vec3( 0, 0,-1)));

        Mesh fragment;
        if (MeshClipper::clipAgainstAll(mesh, planes, fragment)
            && !fragment.empty())
        {
            fragment.recomputeNormals();
            outFragments.emplace_back(std::move(fragment));
        }
    }
    return outFragments.size() - before;
}

} // namespace destruct
