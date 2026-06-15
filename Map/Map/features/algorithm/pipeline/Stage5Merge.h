#pragma once

#include "RoutePipelineTypes.h"

namespace RouteAlgo {

class RoutePipelineDebug;

// ---------------------------------------------------------------------------
// Stage 5 — Merge
// ---------------------------------------------------------------------------
// Post-merges raw graph components into fewer "logical islands". In addition
// to centroid / row-gap gates, boundary agreement uses chord ends projected
// onto `lineDir` (left = smaller dot, right = larger):
//   * Stacked along the sweep (same corridor): left shells match and right
//     shells match along the contour (<= one corner between anchors).
//   * Side-by-side across the sweep: the facing ends match — right(A) with
//     left(B) when centroid(A) is left of centroid(B) in `normal`, else
//     left(A) with right(B) — again <= one corner along the same path.
// Shell anchors are collected from every stripe segment in the component so
// mid-range rows (not only min/max row) still contribute.
//
// Tiny fragments produced in narrow/angled regions get absorbed into a neighbor
// when:
//   * row gap is small AND centroid-distance fits the merge envelope, OR
//   * one side is tiny, sits on the same outer/hole path, and a direct
//     interior connector exists between centroids ("bypass merge"),
//   * and the contour-edge touch condition above holds.
//
// Outputs `components` and a node→component map (`nodeComponent`).
bool runStageMerge(const RoutePipelineInput& input, RoutePipelineState& state,
                   RoutePipelineDebug& debug);

}  // namespace RouteAlgo
