#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 8 — Stitch
// ---------------------------------------------------------------------------
// Wraps `RouteAlgo::stitchIslands`. Walks all islands and connects them
// into a single polyline `stitchedProj`, recording each transition in
// `islandLinks` for visualization.
bool runStageStitch(const RoutePipelineInput& input, RoutePipelineState& state,
                    RoutePipelineDebug& debug);

}  // namespace RouteAlgo
