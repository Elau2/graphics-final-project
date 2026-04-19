#pragma once
// Shader.h
//
// Thin RAII wrapper around an OpenGL program built from a vertex /
// fragment shader pair. Reports compile and link errors to stderr with
// the offending file path so debugging a shader typo is quick.

#include <glm/glm.hpp>
#include <string>
#include <cstdint>

namespace destruct {

class Shader {
public:
    Shader() = default;
    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& o) noexcept;
    Shader& operator=(Shader&& o) noexcept;

    // Compile + link. Returns false on error (error message already logged
    // to stderr). On failure, any previously linked program is destroyed.
    bool loadFromFiles(const std::string& vertPath,
                       const std::string& fragPath);

    // Same thing but from in-memory source strings.
    bool loadFromSource(const std::string& vertSrc,
                        const std::string& fragSrc,
                        const std::string& label = "inline");

    void use() const;

    // Uniform helpers. No-op if the uniform is absent (useful when a
    // shader optimises a uniform away).
    void setMat4 (const char* name, const glm::mat4& m) const;
    void setMat3 (const char* name, const glm::mat3& m) const;
    void setVec3 (const char* name, const glm::vec3& v) const;
    void setVec4 (const char* name, const glm::vec4& v) const;
    void setFloat(const char* name, float v) const;
    void setInt  (const char* name, int v) const;

    uint32_t id() const { return program_; }

private:
    uint32_t program_ = 0;
};

} // namespace destruct
