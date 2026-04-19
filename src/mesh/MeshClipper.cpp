#include "mesh/MeshClipper.h"

#include <glm/glm.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>

namespace destruct {

namespace {

// Key for deduplicating cut-vertices on an edge (a, b) regardless of
// the direction in which the edge was traversed.
struct EdgeKey {
    uint32_t lo = 0, hi = 0;
    bool operator==(const EdgeKey& o) const { return lo == o.lo && hi == o.hi; }
};
struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& k) const noexcept {
        return std::hash<uint64_t>()(
            (static_cast<uint64_t>(k.lo) << 32) | static_cast<uint64_t>(k.hi));
    }
};

} // anonymous namespace

bool MeshClipper::clip(const Mesh& input, const Plane& plane, Mesh& output) {
    output.clear();
    if (input.empty()) return false;

    // Small epsilon avoids generating zero-length edges when a vertex lies
    // exactly on the plane. Vertices within +/-EPS are treated as "on".
    constexpr float EPS = 1e-6f;

    const auto& Vin = input.vertices;
    const auto& Tin = input.triangles;

    // ---- 1. Classify vertices by signed distance to the plane.
    std::vector<float> dist(Vin.size());
    std::vector<int>   cls(Vin.size());   // -1 inside, 0 on plane, +1 outside
    bool anyInside = false, anyOutside = false;
    for (std::size_t i = 0; i < Vin.size(); ++i) {
        float d = plane.signedDistance(Vin[i]);
        dist[i] = d;
        cls[i]  = (d > EPS) ? 1 : (d < -EPS ? -1 : 0);
        if (cls[i] <= 0) anyInside  = true;
        if (cls[i] >  0) anyOutside = true;
    }

    // ---- 2. Trivial cases.
    if (!anyInside)  return false;        // everything is clipped away
    if (!anyOutside) {                    // nothing to clip; copy and return
        output = input;
        return true;
    }

    // ---- 3. Copy kept (inside + on-plane) vertices to the output.
    std::vector<int> vmap(Vin.size(), -1);
    for (std::size_t i = 0; i < Vin.size(); ++i) {
        if (cls[i] <= 0) {
            vmap[i] = static_cast<int>(output.vertices.size());
            output.vertices.push_back(Vin[i]);
        }
    }

    // ---- 4. On-demand generator for a cut-vertex on edge (a, b).
    // Dedup per unordered edge so that neighbouring triangles share it.
    std::unordered_map<EdgeKey, uint32_t, EdgeKeyHash> cutOnEdge;

    auto cutVertex = [&](uint32_t a, uint32_t b) -> uint32_t {
        EdgeKey key{ std::min(a, b), std::max(a, b) };
        if (auto it = cutOnEdge.find(key); it != cutOnEdge.end()) return it->second;
        // Linear interpolation along the edge for the plane intersection.
        // t is the fraction from a toward b where dist becomes zero.
        float denom = dist[a] - dist[b];
        float t     = (std::abs(denom) < 1e-20f) ? 0.5f : dist[a] / denom;
        t = std::clamp(t, 0.0f, 1.0f);
        glm::vec3 p  = Vin[a] + t * (Vin[b] - Vin[a]);
        uint32_t idx = static_cast<uint32_t>(output.vertices.size());
        output.vertices.push_back(p);
        cutOnEdge.emplace(key, idx);
        return idx;
    };

    // ---- 5. Clip each triangle; collect cap segments for later stitching.
    // A "cap segment" is a directed pair (u -> v) on the cut plane. We pick
    // the direction that follows the kept region's CCW boundary walk on the
    // original surface; in step 7 we verify/repair cap winding so correctness
    // does not depend on getting the direction exactly right here.
    std::vector<std::pair<uint32_t, uint32_t>> capSegs;
    capSegs.reserve(Tin.size() / 2);

    for (const Tri& t : Tin) {
        uint32_t idx[3] = { t.a, t.b, t.c };
        int      c[3]   = { cls[t.a], cls[t.b], cls[t.c] };

        const int nOut = (c[0] > 0) + (c[1] > 0) + (c[2] > 0);
        const int nIn  = 3 - nOut;

        if (nOut == 0) {
            // Entire triangle kept.
            output.triangles.emplace_back(vmap[idx[0]], vmap[idx[1]], vmap[idx[2]]);
            continue;
        }
        if (nIn == 0) continue;  // entire triangle removed

        if (nOut == 1) {
            // One vertex outside. Rotate so the outside vertex is position 2.
            int k = (c[0] > 0) ? 0 : (c[1] > 0 ? 1 : 2);
            int iA = (k + 1) % 3, iB = (k + 2) % 3, iO = k;
            uint32_t A = idx[iA], B = idx[iB], O = idx[iO];
            uint32_t pBO = cutVertex(B, O);
            uint32_t pOA = cutVertex(O, A);
            // Kept region is the quad (A, B, pBO, pOA); fan-triangulate it.
            output.triangles.emplace_back(vmap[A], vmap[B], pBO);
            output.triangles.emplace_back(vmap[A], pBO,    pOA);
            // Cap seam segment along this triangle.
            capSegs.emplace_back(pBO, pOA);
        } else {
            // Two outside, one inside. Put the inside at position 0.
            int k = (c[0] <= 0) ? 0 : (c[1] <= 0 ? 1 : 2);
            int iA = k, iB = (k + 1) % 3, iC = (k + 2) % 3;
            uint32_t A = idx[iA], B = idx[iB], C = idx[iC];
            uint32_t pAB = cutVertex(A, B);
            uint32_t pCA = cutVertex(C, A);
            output.triangles.emplace_back(vmap[A], pAB, pCA);
            capSegs.emplace_back(pAB, pCA);
        }
    }

    // ---- 6. Stitch cap segments into closed loops via adjacency.
    // For a closed manifold input, every cap-vertex has exactly one outgoing
    // and one incoming segment.
    if (!capSegs.empty()) {
        std::unordered_map<uint32_t, uint32_t> next;
        next.reserve(capSegs.size() * 2);
        for (auto& s : capSegs) next[s.first] = s.second;

        std::unordered_set<uint32_t> visited;
        visited.reserve(next.size() * 2);

        // ---- 7. For each loop, fan-triangulate from its centroid, then
        // verify the cap normals agree with plane.normal. Flip if needed.
        for (auto& s : capSegs) {
            if (visited.count(s.first)) continue;

            std::vector<uint32_t> loop;
            uint32_t cur   = s.first;
            uint32_t start = cur;
            while (!visited.count(cur)) {
                visited.insert(cur);
                loop.push_back(cur);
                auto it = next.find(cur);
                if (it == next.end()) break;        // open loop - shouldn't happen
                cur = it->second;
                if (cur == start) break;            // closed the loop
            }
            if (loop.size() < 3) continue;          // degenerate

            // Centroid for fan triangulation.
            glm::vec3 centroid(0.0f);
            for (uint32_t v : loop) centroid += output.vertices[v];
            centroid /= static_cast<float>(loop.size());
            uint32_t cIdx = static_cast<uint32_t>(output.vertices.size());
            output.vertices.push_back(centroid);

            std::size_t startTri = output.triangles.size();
            for (std::size_t i = 0; i < loop.size(); ++i) {
                uint32_t a = loop[i];
                uint32_t b = loop[(i + 1) % loop.size()];
                output.triangles.emplace_back(cIdx, a, b);
            }

            // Winding check: pick the largest-area cap triangle (in case some
            // are tiny/degenerate) and test its face normal vs plane.normal.
            // Flip every cap triangle together if necessary.
            float bestArea2 = 0.0f;
            glm::vec3 bestN(0.0f);
            for (std::size_t i = startTri; i < output.triangles.size(); ++i) {
                const Tri& tr = output.triangles[i];
                glm::vec3 n  = glm::cross(
                    output.vertices[tr.b] - output.vertices[tr.a],
                    output.vertices[tr.c] - output.vertices[tr.a]);
                float a2 = glm::dot(n, n);
                if (a2 > bestArea2) { bestArea2 = a2; bestN = n; }
            }
            if (bestArea2 > 0.0f && glm::dot(bestN, plane.normal) < 0.0f) {
                for (std::size_t i = startTri; i < output.triangles.size(); ++i) {
                    std::swap(output.triangles[i].b, output.triangles[i].c);
                }
            }
        }
    }

    // Rebuild vertex normals; without this, lighting on the cap would be wrong.
    output.recomputeNormals();
    return !output.triangles.empty();
}

bool MeshClipper::clipAgainstAll(const Mesh& input,
                                 const std::vector<Plane>& planes,
                                 Mesh& output) {
    if (planes.empty()) { output = input; return !input.empty(); }

    Mesh a = input;
    Mesh b;
    for (const Plane& p : planes) {
        if (!clip(a, p, b)) { output.clear(); return false; }
        std::swap(a, b);
        b.clear();
    }
    output = std::move(a);
    return !output.empty();
}

} // namespace destruct
