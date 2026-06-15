#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 3 — Filter
// ---------------------------------------------------------------------------
// Drops stripes that are too short to be useful (`tuning.minUsefulSegmentLength`)
// unless they have valid neighbors above and below. Then re-indexes `row` so
// that `row+1` always means "next physically adjacent stripe by `d`", instead
// of the artificial sweep-traversal order. Finally builds rowsSegments (a flat
// per-row container used by Stage 4 to walk neighbors quickly).
//
// Inputs:
//   * rawStripes (Stage 2), tuning, stepMeters
// Outputs:
//   * stripes              — kept stripes, re-rowed and sorted by (row, d)
//   * droppedStripes       — for visualization only (red overlay)
//   * rowCount             — number of distinct rows after re-indexing
//   * rowsSegments         — per-row vector of stripes
bool runStageFilter(const RoutePipelineInput& input, RoutePipelineState& state,
                    RoutePipelineDebug& debug);

}  // namespace RouteAlgo
