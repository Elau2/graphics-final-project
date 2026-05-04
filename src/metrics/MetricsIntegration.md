# Wiring FractureMetrics into your Application

## 1. File placement

```
src/
  metrics/
    FractureMetrics.h
    FractureMetrics.cpp
    MetricsOverlay.h
    MetricsOverlay.cpp
```

Add to CMakeLists.txt:
```cmake
target_sources(destruct_core PRIVATE
    src/metrics/FractureMetrics.cpp
    src/metrics/MetricsOverlay.cpp
)
```

---

## 2. In Application.h — add two members

```cpp
#include "metrics/FractureMetrics.h"
#include "metrics/MetricsOverlay.h"

// inside class Application:
destruct::FractureMetrics metrics_;
destruct::FractureMetrics::Thresholds thresholds_; // use defaults or tweak
bool settling_ = false;
```

---

## 3. After fracture — call computeFractureMetrics()

Wherever you call `voronoiFracture()` or `uniformFracture()`, add:

```cpp
// Voronoi example:
auto seeds = voronoiFracture(mesh_, params_, fragments_);

destruct::computeFractureMetrics(
    mesh_,           // original mesh
    fragments_,      // output of fracture
    params_.numSeeds,// seeds requested
    metrics_);

settling_ = true;   // start tracking physics

// Optional: log to console
printf("[Metrics] %s\n", metrics_.summary().c_str());
```

For uniform fracture, pass `params.nx * params.ny * params.nz` as requested.

---

## 4. In the physics step loop — call updatePhysicsMetrics()

```cpp
void Application::physicsStep(float dt)
{
    world_.step(dt);

    if (settling_) {
        destruct::updatePhysicsMetrics(dt, world_, metrics_);

        // Check if all bodies are asleep
        bool allAsleep = std::all_of(
            world_.bodies.begin(), world_.bodies.end(),
            [](const auto& b){ return b->isStatic() || b->sleeping; });

        if (allAsleep) settling_ = false; // freeze settling time
    }
}
```

---

## 5. In the render loop — draw the overlay

```cpp
void Application::drawUI()
{
    ImGui::NewFrame();
    // ... your existing UI ...

    destruct::drawMetricsOverlay(metrics_, thresholds_, settling_);

    ImGui::Render();
}
```

---

## 6. Reset on refracture

```cpp
void Application::onRefracture()
{
    metrics_ = destruct::FractureMetrics{};
    settling_ = false;
    world_.wakeAll();
    // ... rest of refracture logic ...
}
```

---

## What the overlay shows

| Metric | Pass condition | Why |
|---|---|---|
| Fragment yield | ≥ 80% of seeds | Empty cells = seed placement problem |
| Volume conservation | 95–102% of original | <95% = clipping loss; >102% = mesh overlap |
| Uniformity score | ≥ 40% | `1 - stddev/mean`; Voronoi should be reasonably even |
| Convexity ratio | ≥ 90% of fragments | Voronoi cells are always convex; failures = clipper bug |
| Mean aspect ratio | ≤ 4.0 | Needle fragments destabilise physics |
| Settling time | ≤ 5 s | Fragments should come to rest quickly |
| Final overlap error | ≤ 0.01 m | Physics resolution quality at rest |

Thresholds are in `FractureMetrics::Thresholds` — tweak any of them for your scene scale.
