#pragma once

#include "../../RouteAlgorithmConfig.h"
#include "../RouteConnectorPolicy.h"
#include "../RouteGeometryUtils.h"
#include "../RouteIslandBuilder.h"
#include "../RouteIslandStitcher.h"

#include "../../../clipper/clipper.hpp"

#include <QGeoView/QGVMap.h>
#include <QPointF>
#include <QString>
#include <QVector>

#include <memory>
#include <utility>
#include <vector>

namespace RouteAlgo {

// ---------------------------------------------------------------------------
// Stage identifiers
// ---------------------------------------------------------------------------
//
// The pipeline runs strictly in this order. Each stage owns a clearly
// delineated section of the route algorithm and writes its output into the
// shared `RoutePipelineState`. A stage may consume any data produced by a
// previous stage but never reaches forward.
enum class PipelineStage : int {
  Prepare = 0,
  Sweep,
  Filter,
  Graph,
  Merge,
  Routes,
  Group,
  Stitch,
  Approach,
  Count,
};

QString pipelineStageName(PipelineStage stage);

// ---------------------------------------------------------------------------
// Tuning derived from RouteAlgorithmConfig + stepMeters once per build.
// ---------------------------------------------------------------------------
struct RouteTuning {
  double minUsefulSegmentLength = 0.0;
  double intraStrict = 0.0;
  double invalidConnectorCost = 1e12;
};

// ---------------------------------------------------------------------------
// Single forward pass of the snake inside one component, before grouping
// into IslandRoute objects.
// ---------------------------------------------------------------------------
struct RoutedChunk {
  QVector<QPointF> pts;
  int compId = -1;
  int phase = 0;
  int minRow = -1;
  int maxRow = -1;
};

// Pair (compId, phase) -> island index. Used by debug overlays.
struct IslandKey {
  int compId = -1;
  int phase = 0;
};

// ---------------------------------------------------------------------------
// Inputs passed to the pipeline. Read-only for stages.
// ---------------------------------------------------------------------------
struct RoutePipelineInput {
  // Map view / drawing context. Stages don't draw; the visualizer does.
  QGVMap* map = nullptr;
  // Pre-projected contour, in projection coordinates.
  QVector<QPointF> projContour;
  // User-defined cut-out polygons in projection coordinates. They are
  // subtracted from `projContour` before the inset/route stages run.
  QVector<QVector<QPointF>> projCutouts;
  // Start anchor (projection) where the user wants the route to begin.
  QPointF startProj;
  // Optional end anchor (in projection); unset is conveyed by a zero point.
  QPointF endProj;
  bool hasEndPoint = false;
  // Build parameters from the UI.
  double stepMeters = 1.0;
  double angleDegrees = 0.0;
  double contourOffset = 0.0;
  bool rightSide = true;
  // Direction in which the very first row of the very first island is
  // traversed: `true` -> along `+lineDir`, `false` -> along `-lineDir`.
  // The remaining rows zig-zag from there. The next island onwards is picked
  // freely by the stitcher (any of its 4 corners).
  bool forwardAlongLineFirst = true;
  RouteAlgorithmConfig config;
  bool routeDebug = false;
};

// ---------------------------------------------------------------------------
// All intermediate state produced by the pipeline. Visible to all stages, the
// orchestrator, the visualizer and the UI debug tabs.
// ---------------------------------------------------------------------------
struct RoutePipelineState {
  // Stage 1 (Prepare) ------------------------------------------------------
  double stepMeters = 1.0;       // copied from input for downstream consumers
  ClipperLib::Paths workingRegion; // contour − cutouts (before inset)
  RegionGeometry workingGeo;       // geometry for подводка/возврат along outer boundary
  ClipperLib::Paths insetRegion;
  RegionGeometry insetGeo;
  QPointF nearestPt;
  QPointF lineDir;
  QPointF normal;
  RouteTuning tuning;
  std::shared_ptr<RouteConnectorPolicy> connectorPolicy;

  // Stage 2 (Sweep) --------------------------------------------------------
  std::vector<StripeSegment> rawStripes;
  int sweepRowCount = 0;
  int passesExecuted = 0;
  ClipperLib::Paths residualRegion;

  // Stage 3 (Filter) -------------------------------------------------------
  std::vector<StripeSegment> stripes;     // kept stripes (re-rowed, sorted)
  std::vector<StripeSegment> droppedStripes;
  int rowCount = 0;
  std::vector<std::vector<StripeSegment>> rowsSegments;

  // Stage 4 (Graph) --------------------------------------------------------
  std::vector<IslandGraphNode> nodes;
  std::vector<std::vector<int>> rawComponents;

  // Stage 5 (Merge) --------------------------------------------------------
  std::vector<std::vector<int>> components;     // merged components
  std::vector<int> nodeComponent;               // node id -> merged comp id

  // Stage 6 (Routes) -------------------------------------------------------
  std::vector<RoutedChunk> routedChunks;
  int acceptedSegments = 0;

  // Stage 7 (Group) --------------------------------------------------------
  std::vector<IslandRoute> islands;             // chunks grouped by (comp, phase)

  // Stage 8 (Stitch) -------------------------------------------------------
  StitchResult stitchResult;
  QVector<QPointF> stitchedProj;
  std::vector<IslandLink> islandLinks;

  // Stage 9 (Approach) -----------------------------------------------------
  QVector<QPointF> approachProj;
  QVector<QPointF> returnProj;
  QVector<QPointF> fullRouteProj;
  QVector<QGV::GeoPos> routeGeo;
};

}  // namespace RouteAlgo
