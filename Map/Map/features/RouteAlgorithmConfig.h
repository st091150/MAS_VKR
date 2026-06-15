#pragma once

// Minimal, single-purpose parameters used by the parallel route algorithm.
// Each value is documented and kept only if it has a clear, observable effect.
struct RouteAlgorithmConfig {
  // 1) Segment filtering after sweep.
  // Drop fragments shorter than max(minUsefulSegmentLengthMin, step * factor).
  double minUsefulSegmentLengthFactor = 0.65;
  double minUsefulSegmentLengthMin = 0.8;

  // 2) Graph scoring: edges between adjacent rows.
  // score = w_along * gapAlong + w_across * gapAcross + w_angle * angleDiff
  double scoreWeightAlong = 1.0;
  double scoreWeightAcross = 1.8;
  double scoreWeightAngle = 0.35;
  // Edge accepted only if score <= scoreStrictThreshold.
  double scoreStrictThreshold = 3.4;

  // Local geometry gate (overlap / centerline tolerance) as factor of step.
  double geometryGateSlackFactor = 1.0;

  // Cap on outgoing edges per node (sparseness).
  int topKNeighbors = 4;

  // 3) Component post-merge for tiny stranded fragments.
  int mergeRowGapLimit = 3;
  double mergeCentroidAlongFactor = 3.4;
  double mergeCentroidAcrossFactor = 4.8;
  int mergeTinySizeLimit = 18;

  // 4) Intra-island snake split threshold.
  // If connector cost exceeds this, snake within island is split into chunks.
  double intraStrictFactor = 2.2;
  double intraStrictMin = 12.0;

  // 5) Connector policy.
  // Direct cross-field shortcut allowed only if its straight-line length
  // is <= this many meters AND the segment lies fully inside the working
  // region. Anything longer falls back to a contour walk, which is the only
  // legal alternative for an agricultural scout robot. Lower this to make
  // the route stick more strictly to rows + contour.
  double shortDirectConnectorMaxMeters = 5.0;
  // Upper bound for short direct hops between stripes on the same island
  // (also limited by ~2.5 * step in `RouteConnectorPolicy`).
  double intraIslandDirectConnectorMaxMeters = 5.0;
  // Penalty multiplier for long inter-island hops in stitching (helps the
  // stitcher prefer nearby islands when several are reachable).
  double longJumpPenaltyWeight = 1.05;
  double invalidConnectorCost = 1e12;
};

RouteAlgorithmConfig defaultRouteAlgorithmConfig();
