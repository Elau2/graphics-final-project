#pragma once

#include "metrics/FractureMetrics.h"

namespace destruct {

void drawMetricsOverlay(const FractureMetrics& metrics,
                        const FractureMetrics::Thresholds& thresholds = {},
                        bool settling = true);

} // namespace destruct
