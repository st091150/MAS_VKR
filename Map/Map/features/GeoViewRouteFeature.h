#pragma once

#include "algorithm/pipeline/RoutePipelineDebug.h"
#include "algorithm/pipeline/RoutePipelineTypes.h"

#include "clipper/clipper.hpp"

#include <QString>
#include <QVector>
#include <vector>

namespace QGV {
class GeoPos;
}
class QPointF;
class QPen;
class QColor;
class QGVItem;
class QGVLayer;
class GeoViewWidget;

class GeoViewRouteFeature {
 public:
  static QPointF moveAlongContour(GeoViewWidget& view, const ClipperLib::Path& contour,
                                  const QPointF& startPt, double stepMeters, bool forward);

  // Runs the full pipeline. If `debugMode` is true the layer is rendered with
  // the debug overlay for the requested `visualizationStage` (defaults to the
  // final stage). Otherwise the operational presentation is drawn.
  // The route remains accessible via `lastRouteGeo()` / `view.mState.routePoints`.
  static void buildRouteWithAngleForCustomRoute(
      GeoViewWidget& view, double stepMeters, double angleDegrees, double contourOffset,
      const QGV::GeoPos& startPointParam, const QGV::GeoPos& endPointParam, bool rightSide,
      bool debugMode = false,
      RouteAlgo::PipelineStage visualizationStage = RouteAlgo::PipelineStage::Approach);

  // Re-renders the route layer using the previously computed pipeline state,
  // showing the overlay for `stage`. Cheap (no recomputation). `stepLimit`
  // truncates the in-stage progression: < 0 means "show everything".
  static void setVisualizationStage(GeoViewWidget& view, RouteAlgo::PipelineStage stage,
                                    int stepLimit = -1);
  // Maximum number of step-positions the slider can take inside a stage
  // (1 if there's nothing meaningful to step through).
  static int stageStepCount(RouteAlgo::PipelineStage stage);

  // Pipeline result accessors (UI debug tabs read from these).
  static const std::vector<RouteAlgo::StageInfo>& lastPipelineStages();
  static QString lastBuildStatus();
  static bool lastBuildSuccessful();
  // From Prepare metrics (`outerContours` on inset region). 1 = single region.
  static int lastPrepareOuterContourCount();
  // From Stitch (`unreachableIslandIndices`). 0 = all coverage components reached.
  static int lastUnreachableIslandCount();
  static RouteAlgo::PipelineStage lastSuccessfulStage();
  static const QVector<QGV::GeoPos>& lastRouteGeo();

  // Упрощённый режим: параллельные ряды + змейка, без вырезов и без сшивки островов.
  static bool buildBasicParallelCoverageRoute(GeoViewWidget& view, double stepMeters,
                                              double angleDegrees, double offsetFromContour,
                                              const QGV::GeoPos& routeStartPoint,
                                              const QGV::GeoPos& routeEndPoint, bool hasEndPoint,
                                              bool rightSide = true);
  static void handleMapClickStartRoutePoint(GeoViewWidget& view, const QGV::GeoPos& pos);
  static void handleMapClick(GeoViewWidget& view, const QGV::GeoPos& pos);
  static void drawRoute(GeoViewWidget& view, QGVLayer* layer, const QVector<QGV::GeoPos>& pts,
                        const QPen& pen, bool drawArrow, bool replaceExisting);
  static void toggleManualRouteMode(GeoViewWidget& view);

  // Call before `RoutePipelineVisualizer::drawStage` — it clears the layer via
  // `deleteItems()` and would leave dangling `startPointMarker` / `endPointMarker`.
  static void invalidateRouteAnchorMarkers(GeoViewWidget& view);
  static void restoreRouteAnchorMarkers(GeoViewWidget& view, const QGV::GeoPos& startPoint,
                                        const QGV::GeoPos& endPoint, bool hasEndPoint);

 private:
  static void addRouteAnchorMarker(GeoViewWidget& view, const QGV::GeoPos& point,
                                   const QColor& color, QGVItem*& markerSlot);
};
