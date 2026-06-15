#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 6 — Routes
// ---------------------------------------------------------------------------
// Builds snake-shaped traversal chunks per component, in two opposite
// orientations (phase 0 and phase 1). A chunk is a continuous polyline; we
// split into a new chunk whenever the intra-component connector cost exceeds
// `tuning.intraStrict` or the connector policy refuses to bridge the gap.
//
// Output: `routedChunks` (one or more per component × phase).
bool runStageRoutes(const RoutePipelineInput& input, RoutePipelineState& state,
                    RoutePipelineDebug& debug);

}  // namespace RouteAlgo
