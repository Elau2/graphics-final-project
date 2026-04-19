// Shader.cpp
#include "render/Shader.h"

#include <GL/glew.h>
#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <sstream>
#include <iostream>
#include <utility>

namespace destruct {

namespace {

std::string readFile(const std::string& path)
{
    std::ifstream in(path);
    if (!in) {
        std::cerr << "[Shader] failed to open " << path << "\n";
        return {};
    }
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

GLuint compile(GLenum type, const std::string& src, const std::string& label)
{
    GLuint s = glCreateShader(type);
    const char* cstr = src.c_str();
    const GLint len = static_cast<GLint>(src.size());
    glShaderSource(s, 1, &cstr, &len);
    glCompileShader(s);

    GLint ok = GL_FALSE;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint logLen = 0;
        glGetShaderiv(s, GL_INFO_LOG_LENGTH, &logLen);
        std::string log(std::max(logLen, 1), '\0');
        glGetShaderInfoLog(s, logLen, nullptr, log.data());
        std::cerr << "[Shader] compile error in "
                  << (type == GL_VERTEX_SHADER ? "vert " : "frag ")
                  << label << ":\n" << log << "\n";
        glDeleteShader(s);
        return 0;
    }
    return s;
}

} // namespace

Shader::~Shader()
{
    if (program_) glDeleteProgram(program_);
}

Shader::Shader(Shader&& o) noexcept : program_(o.program_) { o.program_ = 0; }
Shader& Shader::operator=(Shader&& o) noexcept
{
    if (this != &o) {
        if (program_) glDeleteProgram(program_);
        program_   = o.program_;
        o.program_ = 0;
    }
    return *this;
}

bool Shader::loadFromFiles(const std::string& vertPath,
                           const std::string& fragPath)
{
    std::string v = readFile(vertPath);
    std::string f = readFile(fragPath);
    if (v.empty() || f.empty()) return false;
    return loadFromSource(v, f, vertPath + " / " + fragPath);
}

bool Shader::loadFromSource(const std::string& vertSrc,
                            const std::string& fragSrc,
                            const std::string& label)
{
    GLuint v = compile(GL_VERTEX_SHADER,   vertSrc, label);
    if (!v) return false;
    GLuint f = compile(GL_FRAGMENT_SHADER, fragSrc, label);
    if (!f) { glDeleteShader(v); return false; }

    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDetachShader(p, v);
    glDetachShader(p, f);
    glDeleteShader(v);
    glDeleteShader(f);

    GLint ok = GL_FALSE;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint logLen = 0;
        glGetProgramiv(p, GL_INFO_LOG_LENGTH, &logLen);
        std::string log(std::max(logLen, 1), '\0');
        glGetProgramInfoLog(p, logLen, nullptr, log.data());
        std::cerr << "[Shader] link error in " << label << ":\n" << log << "\n";
        glDeleteProgram(p);
        return false;
    }

    if (program_) glDeleteProgram(program_);
    program_ = p;
    return true;
}

void Shader::use() const
{
    if (program_) glUseProgram(program_);
}

void Shader::setMat4(const char* name, const glm::mat4& m) const
{
    GLint loc = glGetUniformLocation(program_, name);
    if (loc >= 0) glUniformMatrix4fv(loc, 1, GL_FALSE, glm::value_ptr(m));
}
void Shader::setMat3(const char* name, const glm::mat3& m) const
{
    GLint loc = glGetUniformLocation(program_, name);
    if (loc >= 0) glUniformMatrix3fv(loc, 1, GL_FALSE, glm::value_ptr(m));
}
void Shader::setVec3(const char* name, const glm::vec3& v) const
{
    GLint loc = glGetUniformLocation(program_, name);
    if (loc >= 0) glUniform3fv(loc, 1, glm::value_ptr(v));
}
void Shader::setVec4(const char* name, const glm::vec4& v) const
{
    GLint loc = glGetUniformLocation(program_, name);
    if (loc >= 0) glUniform4fv(loc, 1, glm::value_ptr(v));
}
void Shader::setFloat(const char* name, float v) const
{
    GLint loc = glGetUniformLocation(program_, name);
    if (loc >= 0) glUniform1f(loc, v);
}
void Shader::setInt(const char* name, int v) const
{
    GLint loc = glGetUniformLocation(program_, name);
    if (loc >= 0) glUniform1i(loc, v);
}

} // namespace destruct
