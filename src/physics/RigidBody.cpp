// RigidBody.cpp
#include "physics/RigidBody.h"
#include "mesh/MeshProperties.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>

namespace destruct {

std::unique_ptr<RigidBody>
RigidBody::fromMesh(Mesh mesh, float density)
{
    auto rb = std::make_unique<RigidBody>();

    if (mesh.empty()) {
        rb->mass    = 0.0f;
        rb->invMass = 0.0f;
        rb->meshLocal = std::move(mesh);
        return rb;
    }

    MassProperties mp = computeMassProperties(mesh);

    // Guard against degenerate fragments (sliver cells, inside-out meshes).
    const float kMinVolume = 1e-8f;
    if (mp.volume < kMinVolume) {
        rb->mass        = 0.0f;
        rb->invMass     = 0.0f;
        rb->Ibody       = glm::mat3(1.0f);
        rb->invIbody    = glm::mat3(0.0f);
        rb->meshLocal   = std::move(mesh);
        rb->meshLocal.computeAABB(rb->aabbMin, rb->aabbMax);
        return rb;
    }

    // Recenter mesh so its CoM is at the body origin.
    mesh.translate(-mp.centerOfMass);

    const float mass = density * mp.volume;
    // inertiaAboutCoM is for density = 1, so scale by density.
    glm::mat3 Ibody = density * mp.inertiaAboutCoM;

    rb->mass     = mass;
    rb->invMass  = 1.0f / mass;
    rb->Ibody    = Ibody;
    rb->invIbody = glm::inverse(Ibody);
    rb->invIworld = rb->invIbody;

    rb->meshLocal = std::move(mesh);
    rb->meshLocal.computeAABB(rb->aabbMin, rb->aabbMax);

    // Bounding sphere around body origin = CoM.
    float r2 = 0.0f;
    for (const auto& v : rb->meshLocal.vertices) {
        r2 = std::max(r2, glm::dot(v, v));
    }
    rb->boundingRadius = std::sqrt(r2);

    // Starting transform: sitting at the world origin.
    rb->position = glm::vec3(0.0f);
    rb->orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    return rb;
}

void RigidBody::applyForceAtPoint(const glm::vec3& f, const glm::vec3& worldPoint)
{
    forceAccum  += f;
    torqueAccum += glm::cross(worldPoint - position, f);
}

void RigidBody::applyImpulseAtPoint(const glm::vec3& impulse,
                                    const glm::vec3& worldPoint)
{
    if (isStatic()) return;
    // Any external impulse wakes the body. Collision resolution relies on
    // this: when a moving body strikes a sleeping one, the impulse
    // transferred here reactivates the struck body automatically.
    sleeping   = false;
    sleepTimer = 0.0f;
    linearVelocity  += invMass * impulse;
    angularVelocity += invIworld * glm::cross(worldPoint - position, impulse);
}

glm::vec3 RigidBody::bodyToWorld(const glm::vec3& pBody) const
{
    return position + orientation * pBody;
}

glm::vec3 RigidBody::worldToBody(const glm::vec3& pWorld) const
{
    return glm::conjugate(orientation) * (pWorld - position);
}

glm::vec3 RigidBody::pointVelocityWorld(const glm::vec3& pWorld) const
{
    return linearVelocity + glm::cross(angularVelocity, pWorld - position);
}

void RigidBody::refreshInertiaWorld()
{
    if (isStatic()) { invIworld = glm::mat3(0.0f); return; }
    glm::mat3 R = glm::toMat3(orientation);
    invIworld = R * invIbody * glm::transpose(R);
}

glm::mat4 RigidBody::modelMatrix() const
{
    glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 R = glm::toMat4(orientation);
    return T * R;
}

} // namespace destruct
