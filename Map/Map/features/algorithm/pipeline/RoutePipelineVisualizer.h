#pragma once

#include "RoutePipelineTypes.h"

#include <QColor>
#include <QPen>

#include <utility>
#include <vector>

class QGVMap;
class QGVLayer;

namespace RouteAlgo {

// ---------------------------------------------------------------------------
// Renders pipeline state to a QGV layer. Each `drawStage(stage)` shows what
// that stage produced (and any earlier prerequisites it makes sense to keep,
// e.g. inset outline for context). The user picks a stage in the UI tab and
// the visualizer redraws the layer with just that stage's overlay.
// ---------------------------------------------------------------------------
class RoutePipelineVisualizer {
 public:
  RoutePipelineVisualizer(QGVMap* map, QGVLayer* layer, const RoutePipelineState& state,
                          double stepMeters);

  // Clears the layer and draws the visualization for the requested stage.
  // `stepLimit < 0` means "show everything"; otherwise only the first
  // `stepLimit` items of the stage are rendered (per-stage semantics — see
  // `stageStepCount` for what an "item" means at each stage).
  void drawStage(PipelineStage stage, int stepLimit = -1);
  // Number of meaningful steps the stage can be played back over (1 if there
  // is nothing to step through).
  int stageStepCount(PipelineStage stage) const;
  // Operational presentation: stripes hidden, only final route + approach.
  void drawOperational(const QPen& routePen);

 private:
  // Lower-level draw helpers (all in projection coordinates).
  void drawSegment(const QPointF& a, const QPointF& b, const QColor& color, double width);
  void drawSegments(const std::vector<std::pair<QPointF, QPointF>>& segments,
                    const QColor& color, double width);
  void drawPolyline(const QVector<QPointF>& pts, const QPen& pen);
  void drawCross(const QPointF& c, const QColor& color, double size, double width);
  void drawMarker(const QPointF& c, const QColor& color, double sizeMeters);
  void drawTransition(const QPointF& a, const QPointF& b, const QColor& color, double width);

  // Context layer (always drawn): inset outline + start anchor.
  void drawContextOverlay();

  void clear();

  QGVMap* mMap = nullptr;
  QGVLayer* mLayer = nullptr;
  const RoutePipelineState* mState = nullptr;
  double mStepMeters = 1.0;
};

}  // namespace RouteAlgo
