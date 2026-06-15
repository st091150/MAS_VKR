#pragma once

#include "../RouteAlgorithmConfig.h"
#include "RouteGeometryUtils.h"
#include <QPointF>
#include <vector>

namespace RouteAlgo {

struct IslandGraphNode {
  StripeSegment seg;
  int id = -1;
  std::vector<int> adjStrict;  // candidate row↔row+1 edges (debug / top-K)
};

struct IslandGraphBuildResult {
  std::vector<IslandGraphNode> nodes;
  // Each component is a set of node ids. Components are computed on a
  // **cluster graph**: on one sweep row, disjoint stripe intervals along
  // `lineDir` are different clusters and never coalesce into one island,
  // even if a raw segment-level path could zig-zag between them.
  // Intervals are also separated by `StripeSegment::pathIndex` (disjoint
  // inset outers after cutouts) and by exclusion-boundary touch (cutout /
  // concave pocket) so row clustering never merges two real islands.
  std::vector<std::vector<int>> components;
};

IslandGraphBuildResult buildIslandGraph(const std::vector<std::vector<StripeSegment>>& rowsSegments,
                                        int rowCount, const QPointF& lineDir, const QPointF& normal,
                                        double stepMeters, const RouteAlgorithmConfig& config,
                                        const RegionGeometry& insetGeo,
                                        const RegionGeometry& workingGeo, bool routeDebug);

}  // namespace RouteAlgo
