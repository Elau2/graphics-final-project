cmake_minimum_required(VERSION 3.16)
project(GeometryDestructionSimulator LANGUAGES CXX C)

# ---------- General settings ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
    add_compile_options(/W3 /permissive- /Zc:__cplusplus)
else()
    add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
endif()

# Copy shaders next to the binary so the runtime can find them.
set(SHADER_OUT_DIR ${CMAKE_BINARY_DIR}/shaders)
file(MAKE_DIRECTORY ${SHADER_OUT_DIR})
file(COPY ${CMAKE_SOURCE_DIR}/shaders/ DESTINATION ${SHADER_OUT_DIR})

# ---------- Dependencies ----------
include(FetchContent)

# GLFW (window + input)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# GLM (header-only math)
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        1.0.1
)
FetchContent_MakeAvailable(glm)

# GLEW - prefer system install; fall back to FetchContent build
find_package(GLEW QUIET)
if(NOT GLEW_FOUND)
    message(STATUS "System GLEW not found; fetching glew-cmake instead.")
    FetchContent_Declare(
        glew
        GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
        GIT_TAG        glew-cmake-2.2.0
    )
    FetchContent_MakeAvailable(glew)
    add_library(GLEW::GLEW ALIAS libglew_static)
endif()

find_package(OpenGL REQUIRED)

# ---------- Our library + executable ----------
set(CORE_SOURCES
    src/mesh/Mesh.cpp
    src/mesh/OBJLoader.cpp
    src/mesh/MeshClipper.cpp
    src/mesh/MeshProperties.cpp
    src/fracture/VoronoiFracture.cpp
    src/fracture/UniformFracture.cpp
    src/physics/RigidBody.cpp
    src/physics/PhysicsWorld.cpp
    src/render/Shader.cpp
    src/render/Camera.cpp
    src/render/GPUMesh.cpp
    src/render/Renderer.cpp
    src/app/Application.cpp
    src/app/TimestepRecorder.cpp
)

add_library(destruct_core STATIC ${CORE_SOURCES})
target_include_directories(destruct_core PUBLIC src)
target_compile_definitions(destruct_core PUBLIC
    GLM_ENABLE_EXPERIMENTAL
)
target_link_libraries(destruct_core PUBLIC
    glfw
    glm::glm
    GLEW::GLEW
    OpenGL::GL
)

add_executable(destruct src/main.cpp)
target_link_libraries(destruct PRIVATE destruct_core)
