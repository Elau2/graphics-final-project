#pragma once
// TimestepRecorder.h
//
// Records per-body transforms at every simulated frame so the user can
// scrub backwards/forwards through the simulation (a proposal deliverable).
//
// We deliberately only record the body *state* (position + orientation),
// not forces/velocities/contacts. Replaying means restoring positions
// and orientations directly; the simulation is NOT re-run. This is much
// cheaper both in storage and CPU, and it's all the user wants ("watch
// the pieces fly back together") - but it does mean velocities after a
// scrub are stale. `Application` handles this by zeroing velocities when
// you seek, so if you unpause mid-scrub the simulation restarts cleanly
// from rest.

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <memory>
#include <cstddef>

namespace destruct {

class RigidBody;

struct RBState {
    glm::vec3 position{0.0f};
    glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
};

class TimestepRecorder {
public:
    // Maximum frames retained. Older frames fall off the front when full
    // so the buffer behaves like a ring. Default: ~30 seconds at 60 Hz.
    std::size_t maxFrames = 1800;

    TimestepRecorder() = default;

    // Append a snapshot of the given bodies. If maxFrames is exceeded,
    // the oldest frame is discarded and the playback cursor is adjusted
    // so it still points at the same "real" frame.
    void recordFrame(const std::vector<std::unique_ptr<RigidBody>>& bodies);

    // Wipe everything. Call when the scene topology changes (re-fracture).
    void clear();

    // How many frames currently stored, and the index of the last-recorded.
    std::size_t frameCount() const { return frames_.size(); }

    // Restore all bodies to the state stored at frame index `f`.
    // Silently clamps `f` to [0, frameCount()-1]. Velocities of the
    // restored bodies are zeroed so re-simulation from this frame is
    // well-defined.
    void seek(std::size_t f,
              std::vector<std::unique_ptr<RigidBody>>& bodies) const;

private:
    // frames_[t] has one RBState per body, in the same order as the
    // bodies vector at record time. If the body count changes (e.g.
    // re-fracture), callers must clear() before recording again.
    std::vector<std::vector<RBState>> frames_;
};

} // namespace destruct
