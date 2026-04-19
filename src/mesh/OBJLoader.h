#pragma once
// OBJLoader.h
// A minimal OBJ parser: vertices (v) and triangular faces (f).
// It tolerates:
//   - negative (relative) indices
//   - faces with more than 3 vertices (fan-triangulated)
//   - the "v/t/n" face-index form (we only take the first component)
// It ignores: materials, groups, smoothing, textures. Good enough for the
// Stanford 3D Scanning Repository models and most mesh benchmarks.

#include "mesh/Mesh.h"
#include <string>

namespace destruct {

// Loads `path` into `outMesh`. Returns false on failure (file missing or
// parse error). Normals are always recomputed from the loaded geometry.
bool loadOBJ(const std::string& path, Mesh& outMesh, std::string* errOut = nullptr);

} // namespace destruct
