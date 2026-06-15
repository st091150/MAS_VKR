#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 9 — Approach
// ---------------------------------------------------------------------------
// Adds approach (start → first island) and return (last point → end) legs.
// Connectors follow the outer boundary of the working area (contour − cutouts);
// straight chords through forbidden zones are rejected.
//
// Result lives in `state.fullRouteProj` (projection coords) and `state.routeGeo`
// (geo coords) - the latter is what gets exported as the operational route.
bool runStageApproach(const RoutePipelineInput& input, RoutePipelineState& state,
                      RoutePipelineDebug& debug);

}  // namespace RouteAlgo
