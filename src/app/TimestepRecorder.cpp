// TimestepRecorder.cpp
#include "app/TimestepRecorder.h"
#include "physics/RigidBody.h"

#include <algorithm>

namespace destruct {

void TimestepRecorder::recordFrame(
    const std::vector<std::unique_ptr<RigidBody>>& bodies)
{
    std::vector<RBState> snap;
    snap.reserve(bodies.size());
    for (const auto& b : bodies) {
        snap.push_back({ b->position, b->orientation });
    }
    frames_.emplace_back(std::move(snap));

    // Ring behaviour: drop oldest frame when the cap is exceeded.
    if (frames_.size() > maxFrames) {
        frames_.erase(frames_.begin());
    }
}

void TimestepRecorder::clear()
{
    frames_.clear();
}

void TimestepRecorder::seek(std::size_t f,
                            std::vector<std::unique_ptr<RigidBody>>& bodies) const
{
    if (frames_.empty()) return;
    f = std::min(f, frames_.size() - 1);
    const auto& snap = frames_[f];
    const std::size_t n = std::min(snap.size(), bodies.size());
    for (std::size_t i = 0; i < n; ++i) {
        bodies[i]->position        = snap[i].position;
        bodies[i]->orientation     = snap[i].orientation;
        bodies[i]->linearVelocity  = glm::vec3(0.0f);
        bodies[i]->angularVelocity = glm::vec3(0.0f);
        bodies[i]->refreshInertiaWorld();
    }
}

} // namespace destruct
