#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 1 — Prepare
// ---------------------------------------------------------------------------
// Inputs (RoutePipelineInput): contour, contourOffset, angle, step, start.
// Outputs (RoutePipelineState):
//   * insetRegion        — Clipper paths after offset+union
//   * insetGeo           — Boost+Clipper geometry view (with cached perimeters)
//   * nearestPt          — closest point on the working area to startProj
//   * lineDir, normal    — sweep direction and its perpendicular
//   * tuning             — derived constants (minUsefulSegmentLength, intraStrict)
//   * connectorPolicy    — shared connector cost/append logic (used by 6, 8, 9)
//
// Returns true on success; false signals a fatal precondition violation
// (empty inset, zero direction, etc.) and the pipeline aborts.
bool runStagePrepare(const RoutePipelineInput& input, RoutePipelineState& state,
                     RoutePipelineDebug& debug);

}  // namespace RouteAlgo
