#include "mesh/OBJLoader.h"

#include <fstream>
#include <sstream>
#include <vector>

namespace destruct {

// Parse the index of an OBJ face token "v[/vt][/vn]". We only care about v.
// Returns the 0-based vertex index, or -1 on parse failure.
static int parseIndex(const std::string& token, int nVerts) {
    if (token.empty()) return -1;
    // Everything up to the first '/' is the vertex index.
    std::size_t slash = token.find('/');
    std::string head  = (slash == std::string::npos) ? token : token.substr(0, slash);
    int idx;
    try { idx = std::stoi(head); }
    catch (...) { return -1; }
    if (idx > 0) return idx - 1;        // OBJ is 1-based
    if (idx < 0) return nVerts + idx;   // negative = relative
    return -1;
}

bool loadOBJ(const std::string& path, Mesh& outMesh, std::string* errOut) {
    std::ifstream in(path);
    if (!in) {
        if (errOut) *errOut = "cannot open " + path;
        return false;
    }
    outMesh.clear();

    std::string line;
    while (std::getline(in, line)) {
        // Trim leading whitespace.
        std::size_t start = line.find_first_not_of(" \t\r");
        if (start == std::string::npos || line[start] == '#') continue;

        std::istringstream iss(line.substr(start));
        std::string cmd;
        iss >> cmd;
        if (cmd == "v") {
            glm::vec3 v;
            iss >> v.x >> v.y >> v.z;
            outMesh.vertices.push_back(v);
        } else if (cmd == "f") {
            // Read every face token in the line, then fan-triangulate.
            std::vector<int> indices;
            std::string tok;
            int nVerts = static_cast<int>(outMesh.vertices.size());
            while (iss >> tok) {
                int idx = parseIndex(tok, nVerts);
                if (idx < 0 || idx >= nVerts) {
                    if (errOut) *errOut = "bad face index: " + tok;
                    return false;
                }
                indices.push_back(idx);
            }
            if (indices.size() < 3) continue;
            for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
                outMesh.triangles.emplace_back(
                    static_cast<uint32_t>(indices[0]),
                    static_cast<uint32_t>(indices[i]),
                    static_cast<uint32_t>(indices[i + 1])
                );
            }
        }
        // Everything else (vn, vt, usemtl, g, s, o, ...) is ignored.
    }

    if (outMesh.vertices.empty() || outMesh.triangles.empty()) {
        if (errOut) *errOut = "no geometry parsed from " + path;
        return false;
    }
    outMesh.recomputeNormals();
    return true;
}

} // namespace destruct
