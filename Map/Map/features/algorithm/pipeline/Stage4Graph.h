#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 4 — Graph
// ---------------------------------------------------------------------------
// Wraps `RouteAlgo::buildIslandGraph`. Builds the strict-edge adjacency among
// neighboring stripes and runs DFS to extract connected components.
//
// Inputs: rowsSegments, rowCount, lineDir, normal, stepMeters, config.
// Outputs:
//   * nodes           — graph nodes (one per stripe)
//   * rawComponents   — DFS components (still pre-merge)
bool runStageGraph(const RoutePipelineInput& input, RoutePipelineState& state,
                   RoutePipelineDebug& debug);

}  // namespace RouteAlgo
