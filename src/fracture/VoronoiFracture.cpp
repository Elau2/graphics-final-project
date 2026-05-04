// VoronoiFracture.cpp
#include "fracture/VoronoiFracture.h"
#include "mesh/MeshClipper.h"
#include "math/Plane.h"

#include <algorithm>
#include <chrono>
#include <future>
#include <random>

namespace destruct {

namespace {

// Small helper: uniform in [a, b).
inline float uniform(std::mt19937& rng, float a, float b) {
    std::uniform_real_distribution<float> dist(a, b);
    return dist(rng);
}

// Gaussian sample via Box-Muller on two uniforms.
inline float gaussian(std::mt19937& rng, float mu, float sigma) {
    std::normal_distribution<float> dist(mu, sigma);
    return dist(rng);
}

} // namespace

// Seed generation
std::vector<glm::vec3>
generateSeeds(const Mesh& mesh, const VoronoiParams& params)
{
    std::vector<glm::vec3> seeds;
    if (mesh.empty() || params.numSeeds <= 0) return seeds;

    uint32_t rngSeed = params.rngSeed;
    if (rngSeed == 0u) {
        auto now = std::chrono::steady_clock::now().time_since_epoch().count();
        rngSeed  = static_cast<uint32_t>(now ^ (now >> 32));
    }
    std::mt19937 rng(rngSeed);

    glm::vec3 bbMin, bbMax;
    mesh.computeAABB(bbMin, bbMax);

    // Small inset so seeds land slightly inside the AABB. Helps avoid
    // degenerate cells with planes tangent to the bounding box.
    const glm::vec3 extent = bbMax - bbMin;
    const glm::vec3 inset  = 0.02f * extent;
    bbMin += inset;
    bbMax -= inset;

    auto sampleOne = [&](void) -> glm::vec3 {
        if (params.useImpactBias) {
            // Gaussian cluster around impactPoint, clamped to AABB so we
            // never generate ridiculous outliers.
            glm::vec3 p{
                gaussian(rng, params.impactPoint.x, params.impactRadius),
                gaussian(rng, params.impactPoint.y, params.impactRadius),
                gaussian(rng, params.impactPoint.z, params.impactRadius)
            };
            return glm::clamp(p, bbMin, bbMax);
        } else {
            return glm::vec3(
                uniform(rng, bbMin.x, bbMax.x),
                uniform(rng, bbMin.y, bbMax.y),
                uniform(rng, bbMin.z, bbMax.z)
            );
        }
    };

    const float minSep2 =
        params.minSeedSeparation * params.minSeedSeparation;
    const bool  usePoisson = params.minSeedSeparation > 0.0f;

    // Bound the number of rejection attempts so the function always returns.
    const int maxAttempts = std::max(2000, params.numSeeds * 50);
    int attempts = 0;

    seeds.reserve(params.numSeeds);
    while (static_cast<int>(seeds.size()) < params.numSeeds
           && attempts < maxAttempts)
    {
        glm::vec3 p = sampleOne();
        ++attempts;

        if (usePoisson) {
            bool tooClose = false;
            for (const auto& q : seeds) {
                glm::vec3 d = p - q;
                if (glm::dot(d, d) < minSep2) { tooClose = true; break; }
            }
            if (tooClose) continue;
        }
        seeds.push_back(p);
    }
    return seeds;
}

// Fracture
std::size_t
voronoiFracture(const Mesh& mesh,
                const std::vector<glm::vec3>& seeds,
                std::vector<Mesh>& outFragments)
{
    if (mesh.empty() || seeds.empty()) return 0;

    const std::size_t before = outFragments.size();

    // Each fragment only reads `mesh` and `seeds` (both const) and writes to
    // a private Mesh — perfectly thread-safe for parallel execution.
    const std::size_t N = seeds.size();
    std::vector<std::future<Mesh>> futures;
    futures.reserve(N);

    for (std::size_t i = 0; i < N; ++i) {
        futures.push_back(std::async(std::launch::async,
            [&mesh, &seeds, i, N]() -> Mesh {
                std::vector<Plane> localPlanes;
                localPlanes.reserve(N - 1);
                for (std::size_t j = 0; j < N; ++j) {
                    if (j != i)
                        localPlanes.push_back(Plane::bisector(seeds[i], seeds[j]));
                }
                Mesh fragment;
                if (MeshClipper::clipAgainstAll(mesh, localPlanes, fragment)
                    && !fragment.empty())
                {
                    fragment.recomputeNormals();
                }
                return fragment;
            }));
    }

    for (auto& f : futures) {
        Mesh fragment = f.get();
        if (!fragment.empty())
            outFragments.emplace_back(std::move(fragment));
    }

    return outFragments.size() - before;
}

std::vector<glm::vec3>
voronoiFracture(const Mesh& mesh,
                const VoronoiParams& params,
                std::vector<Mesh>& outFragments)
{
    auto seeds = generateSeeds(mesh, params);
    voronoiFracture(mesh, seeds, outFragments);
    return seeds;
}

} // namespace destruct
