#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 7 — Group
// ---------------------------------------------------------------------------
// Groups RoutedChunks (Stage 6 output) into one IslandRoute per (compId,
// phase) pair. Multiple chunks of the same key get re-joined via the
// connector policy; if the policy refuses, chunks are joined directly to
// preserve coverage. Also computes a centroid for each island for visual
// markers.
bool runStageGroup(const RoutePipelineInput& input, RoutePipelineState& state,
                   RoutePipelineDebug& debug);

}  // namespace RouteAlgo
