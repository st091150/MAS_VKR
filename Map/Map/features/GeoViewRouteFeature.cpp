#include "GeoViewRouteFeature.h"

#include "RouteAlgorithmConfig.h"
#include "algorithm/RouteConnectorPolicy.h"
#include "algorithm/RouteGeometryUtils.h"
#include "algorithm/pipeline/Stage1Prepare.h"
#include "algorithm/pipeline/Stage2Sweep.h"
#include "algorithm/pipeline/Stage3Filter.h"
#include "algorithm/pipeline/RoutePipeline.h"
#include "algorithm/pipeline/RoutePipelineVisualizer.h"

#include "../GeoViewWidget.h"
#include "../geometry/GeoPolyline.h"
#include "../utils/ClipperUtils.h"
#include <rectangle.h>

#include <QGeoView/Raster/QGVIcon.h>
#include <QGeoView/QGVItem.h>
#include <QGeoView/QGVMap.h>
#include <QColor>
#include <QDebug>
#include <QPen>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace {

constexpr double kDupMergeMeters = 0.02;

// Most recent pipeline result.
RouteAlgo::RoutePipeline gPipeline;
QString gLastBuildStatus;
bool gLastBuildOk = false;
QVector<QGV::GeoPos> gLastRouteGeo;

ClipperLib::Paths makeWorkingRegion(const QVector<QPointF>& projContour) {
  if (projContour.size() < 3) return {};
  ClipperLib::Paths workingRegion;
  workingRegion.push_back(ClipperUtils::toClipPath(projContour));
  ClipperUtils::orientClipperPathOuter(workingRegion.back());
  return ClipperUtils::unionPaths(workingRegion);
}

ClipperLib::Paths makeInsetRegion(const QVector<QPointF>& projContour, double contourOffset) {
  const ClipperLib::Paths workingRegion = makeWorkingRegion(projContour);
  if (workingRegion.empty()) return {};

  ClipperLib::ClipperOffset co;
  co.AddPaths(workingRegion, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
  ClipperLib::Paths insetRaw;
  co.Execute(insetRaw, -contourOffset * ClipperUtils::kClipperScale);
  return ClipperUtils::unionPaths(insetRaw);
}

double dotAlong(const QPointF& p, const QPointF& axis) {
  return p.x() * axis.x() + p.y() * axis.y();
}

std::pair<QPointF, QPointF> orientedStripeEnds(const RouteAlgo::StripeSegment& seg,
                                               bool forwardAlongLine, const QPointF& lineDir) {
  const double ta = dotAlong(seg.a, lineDir);
  const double tb = dotAlong(seg.b, lineDir);
  if (forwardAlongLine) {
    return (ta <= tb) ? std::make_pair(seg.a, seg.b) : std::make_pair(seg.b, seg.a);
  }
  return (ta <= tb) ? std::make_pair(seg.b, seg.a) : std::make_pair(seg.a, seg.b);
}

QVector<QPointF> buildBasicCoverageProj(const QVector<QPointF>& projContour, double stepMeters,
                                        double angleDegrees, double contourOffset,
                                        const QPointF& startProj, bool rightSide,
                                        bool forwardAlongLineFirst) {
  RouteAlgo::RoutePipelineInput input;
  input.projContour = projContour;
  input.stepMeters = stepMeters;
  input.angleDegrees = angleDegrees;
  input.contourOffset = contourOffset;
  input.rightSide = rightSide;
  input.forwardAlongLineFirst = forwardAlongLineFirst;
  input.config = defaultRouteAlgorithmConfig();
  input.startProj = startProj;
  input.routeDebug = false;

  RouteAlgo::RoutePipelineState state;
  RouteAlgo::RoutePipelineDebug debug;
  debug.setConsoleEnabled(false);
  if (!RouteAlgo::runStagePrepare(input, state, debug)) return {};
  if (!RouteAlgo::runStageSweep(input, state, debug)) return {};
  if (!RouteAlgo::runStageFilter(input, state, debug)) return {};
  if (!state.connectorPolicy) return {};

  QVector<QPointF> projRoute;
  auto appendPoint = [&](const QPointF& p) {
    if (projRoute.isEmpty() ||
        std::hypot((p - projRoute.back()).x(), (p - projRoute.back()).y()) > kDupMergeMeters) {
      projRoute.push_back(p);
    }
  };
  auto connectTo = [&](const QPointF& to) -> bool {
    if (projRoute.isEmpty()) {
      appendPoint(to);
      return true;
    }
    if (std::hypot((to - projRoute.back()).x(), (to - projRoute.back()).y()) <= kDupMergeMeters) {
      return true;
    }
    return state.connectorPolicy->appendConnectorProj(
        projRoute, projRoute.back(), to, RouteAlgo::ConnectorHopKind::IntraIslandSnake);
  };

  for (const auto& rowSegs : state.rowsSegments) {
    if (rowSegs.empty()) continue;

    std::vector<RouteAlgo::StripeSegment> segs(rowSegs.begin(), rowSegs.end());
    std::sort(segs.begin(), segs.end(), [&](const RouteAlgo::StripeSegment& a,
                                            const RouteAlgo::StripeSegment& b) {
      return dotAlong(a.mid, state.lineDir) < dotAlong(b.mid, state.lineDir);
    });

    bool rowForward = forwardAlongLineFirst;
    if (!projRoute.isEmpty()) {
      const auto leftEntry = orientedStripeEnds(segs.front(), true, state.lineDir).first;
      const auto rightEntry = orientedStripeEnds(segs.back(), true, state.lineDir).second;
      const QPointF& back = projRoute.back();
      rowForward = std::hypot(leftEntry.x() - back.x(), leftEntry.y() - back.y()) <=
                   std::hypot(rightEntry.x() - back.x(), rightEntry.y() - back.y());
    }
    if (!rowForward) {
      std::reverse(segs.begin(), segs.end());
    }

    for (const auto& seg : segs) {
      const auto ends = orientedStripeEnds(seg, rowForward, state.lineDir);
      if (!connectTo(ends.first)) {
        appendPoint(ends.first);
      }
      appendPoint(ends.second);
    }
  }

  if (projRoute.size() >= 2) {
    const QPointF anchorPt = RouteAlgo::nearestPointOnRegion(state.insetRegion, startProj);
    const double headD = std::hypot((projRoute.front() - anchorPt).x(),
                                    (projRoute.front() - anchorPt).y());
    const double tailD = std::hypot((projRoute.back() - anchorPt).x(),
                                    (projRoute.back() - anchorPt).y());
    if (tailD < headD) {
      std::reverse(projRoute.begin(), projRoute.end());
    }
  }
  return projRoute;
}

// Approach/return along the user-drawn outer contour (no reach-limit; avoids diagonal chords).
bool appendContourBoundaryLeg(QVector<QPointF>& route, const RouteAlgo::RegionGeometry& region,
                              const QPointF& to) {
  if (route.isEmpty()) return false;
  const QPointF from = route.back();
  if (std::hypot((to - from).x(), (to - from).y()) <= kDupMergeMeters) return true;

  const RouteAlgo::BoundaryAnchor fromNear =
      RouteAlgo::nearestBoundaryAnchorOnRegion(region, from);
  const RouteAlgo::BoundaryAnchor toNear = RouteAlgo::nearestBoundaryAnchorOnRegion(region, to);
  if (fromNear.pathIndex < 0 || toNear.pathIndex < 0 ||
      fromNear.pathIndex != toNear.pathIndex) {
    return false;
  }
  const int pi = fromNear.pathIndex;
  const RouteAlgo::BoundaryAnchor fromAnchor = RouteAlgo::nearestBoundaryAnchor(
      region.paths[static_cast<size_t>(pi)], from, pi);
  const RouteAlgo::BoundaryAnchor toAnchor = RouteAlgo::nearestBoundaryAnchor(
      region.paths[static_cast<size_t>(pi)], to, pi);
  if (fromAnchor.segIndex < 0 || toAnchor.segIndex < 0) return false;

  auto appendPoint = [&](const QPointF& p) {
    if (route.isEmpty() ||
        std::hypot((p - route.back()).x(), (p - route.back()).y()) > kDupMergeMeters) {
      route.push_back(p);
    }
  };

  if (std::hypot((fromAnchor.point - from).x(), (fromAnchor.point - from).y()) > kDupMergeMeters) {
    appendPoint(fromAnchor.point);
  }

  const QVector<QPointF> arc = RouteAlgo::buildBoundaryConnector(
      region.paths[static_cast<size_t>(pi)], fromAnchor, toAnchor,
      region.pathPerimeters[static_cast<size_t>(pi)]);
  for (int i = 1; i < arc.size(); ++i) {
    appendPoint(arc[i]);
  }

  appendPoint(to);
  return route.size() >= 2;
}

QVector<QPointF> buildBasicFullRouteProj(const QVector<QPointF>& projContour, double stepMeters,
                                         double angleDegrees, double contourOffset,
                                         const QPointF& startProj, const QPointF& endProj,
                                         bool hasEndPoint, bool rightSide,
                                         bool forwardAlongLineFirst) {
  QVector<QPointF> coverage =
      buildBasicCoverageProj(projContour, stepMeters, angleDegrees, contourOffset, startProj,
                             rightSide, forwardAlongLineFirst);
  if (coverage.size() < 2) return {};

  const ClipperLib::Paths workingRegion = makeWorkingRegion(projContour);
  if (workingRegion.empty()) return coverage;
  const RouteAlgo::RegionGeometry workingGeo = RouteAlgo::buildRegionGeometry(workingRegion);

  QVector<QPointF> fullRoute;
  fullRoute.push_back(startProj);
  if (!appendContourBoundaryLeg(fullRoute, workingGeo, coverage.front())) {
    fullRoute.clear();
  }
  const int covStart = fullRoute.isEmpty() ? 0 : 1;
  for (int i = covStart; i < coverage.size(); ++i) {
    if (fullRoute.isEmpty() ||
        std::hypot((coverage[i] - fullRoute.back()).x(), (coverage[i] - fullRoute.back()).y()) >
            kDupMergeMeters) {
      fullRoute.push_back(coverage[i]);
    }
  }
  const QPointF returnTarget = hasEndPoint ? endProj : startProj;
  appendContourBoundaryLeg(fullRoute, workingGeo, returnTarget);
  return fullRoute;
}

bool geoPointsNear(const QGV::GeoPos& a, const QGV::GeoPos& b) {
  const double dLat = a.latitude() - b.latitude();
  const double dLon = a.longitude() - b.longitude();
  return (dLat * dLat + dLon * dLon) < 1e-14;
}

void appendGeoPoint(QVector<QGV::GeoPos>& route, const QGV::GeoPos& p) {
  if (!route.isEmpty() && geoPointsNear(route.back(), p)) return;
  route.push_back(p);
}

QString summariseStatus(const RouteAlgo::RoutePipeline& pipeline) {
  if (pipeline.succeeded()) {
    const int unreachable = static_cast<int>(
        pipeline.state().stitchResult.unreachableIslandIndices.size());
    const int totalIslands = static_cast<int>(pipeline.state().islands.size());
    const int usedIslands = pipeline.state().stitchResult.usedIslandCount;
    if (unreachable > 0) {
      return QObject::tr("Частичный маршрут: %1 из %2 островов (%3 недостижимых)")
          .arg(usedIslands)
          .arg(totalIslands)
          .arg(unreachable);
    }
    const auto* approach = pipeline.debug().find(RouteAlgo::PipelineStage::Approach);
    if (approach && !approach->message.isEmpty()) return approach->message;
    if (totalIslands > 0) {
      return QObject::tr("Маршрут построен (%1 островов)").arg(totalIslands);
    }
    return QObject::tr("Маршрут построен");
  }
  const auto* fail = pipeline.debug().firstFailure();
  if (fail) {
    return fail->message.isEmpty()
               ? QObject::tr("Сбой на стадии %1").arg(fail->name)
               : QObject::tr("[%1] %2").arg(fail->name, fail->message);
  }
  return QObject::tr("Маршрут не построен");
}

}  // namespace

QPointF GeoViewRouteFeature::moveAlongContour(GeoViewWidget& view,
                                              const ClipperLib::Path& contour,
                                              const QPointF& startPt,
                                              double stepMeters, bool forward) {
  Q_UNUSED(view);
  const int n = static_cast<int>(contour.size());
  if (n < 2) return startPt;

  double minDist = std::numeric_limits<double>::max();
  int segIndex = 0;
  double tOnSeg = 0.0;

  for (int i = 0; i < n; ++i) {
    const QPointF p1 = ClipperUtils::fromClip(contour[i]);
    const QPointF p2 = ClipperUtils::fromClip(contour[(i + 1) % n]);
    const QPointF seg = p2 - p1;
    const QPointF toStart = startPt - p1;
    const double t = std::clamp(
        QPointF::dotProduct(toStart, seg) / QPointF::dotProduct(seg, seg), 0.0, 1.0);
    const QPointF projPt = p1 + seg * t;
    const double dist = std::hypot(projPt.x() - startPt.x(), projPt.y() - startPt.y());
    if (dist < minDist) {
      minDist = dist;
      segIndex = i;
      tOnSeg = t;
    }
  }

  double remainingStep = stepMeters;
  int currentIndex = segIndex;
  double t = tOnSeg;

  while (remainingStep > 0) {
    const QPointF p1 = ClipperUtils::fromClip(contour[currentIndex]);
    const QPointF p2 = ClipperUtils::fromClip(contour[(currentIndex + 1) % n]);
    const QPointF seg = p2 - p1;
    const double segLen = std::hypot(seg.x(), seg.y());
    const double stepOnSeg = forward ? (1.0 - t) * segLen : t * segLen;
    if (remainingStep <= stepOnSeg) {
      const double ratio = forward ? t + remainingStep / segLen : t - remainingStep / segLen;
      return p1 + seg * ratio;
    }
    remainingStep -= stepOnSeg;
    currentIndex = forward ? (currentIndex + 1) % n : (currentIndex - 1 + n) % n;
    t = forward ? 0.0 : 1.0;
  }
  return {0, 0};
}

void GeoViewRouteFeature::buildRouteWithAngleForCustomRoute(
    GeoViewWidget& view, double stepMeters, double angleDegrees, double contourOffset,
    const QGV::GeoPos& startPointParam, const QGV::GeoPos& endPointParam, bool rightSide,
    bool debugMode, RouteAlgo::PipelineStage visualizationStage) {
  gLastBuildOk = false;
  gLastBuildStatus = QObject::tr("Маршрут не построен");
  gLastRouteGeo.clear();

  if (view.mContour->points().size() < 2) {
    gLastBuildStatus = QObject::tr("Не задан контур");
    return;
  }

  RouteAlgo::RoutePipelineInput input;
  input.map = view.mMap;
  input.stepMeters = stepMeters;
  input.angleDegrees = angleDegrees;
  input.contourOffset = contourOffset;
  input.rightSide = rightSide;
  input.config = defaultRouteAlgorithmConfig();
  input.routeDebug = debugMode;
  for (const auto& g : view.mContour->points()) {
    input.projContour.push_back(view.mMap->getProjection()->geoToProj(g));
  }
  input.projCutouts.reserve(view.mCutoutPolygons.size());
  for (const auto& cutout : view.mCutoutPolygons) {
    QVector<QPointF> projCutout;
    projCutout.reserve(cutout.size());
    for (const auto& g : cutout) {
      projCutout.push_back(view.mMap->getProjection()->geoToProj(g));
    }
    input.projCutouts.push_back(std::move(projCutout));
  }
  input.startProj = view.mMap->getProjection()->geoToProj(startPointParam);
  input.hasEndPoint =
      (endPointParam.latitude() != 0.0 || endPointParam.longitude() != 0.0);
  input.endProj =
      input.hasEndPoint ? view.mMap->getProjection()->geoToProj(endPointParam) : input.startProj;

  view.clearRouteLayer();
  const bool ok = gPipeline.run(input);
  gLastBuildOk = ok;
  gLastBuildStatus = summariseStatus(gPipeline);

  RouteAlgo::RoutePipelineVisualizer viz(view.mMap, view.mRouteLayer, gPipeline.state(),
                                         stepMeters);
  if (debugMode) {
    // Pick a stage to show: requested one if it succeeded, otherwise the last
    // successful stage so the user always gets *something* on the map.
    const auto* failure = gPipeline.debug().firstFailure();
    const RouteAlgo::PipelineStage chosen =
        gPipeline.debug().stageOk(visualizationStage) || ok
            ? visualizationStage
            : (failure ? failure->stage : gPipeline.lastSuccessfulStage());
    viz.drawStage(chosen);
  } else if (ok) {
    viz.drawOperational(view.mRouteColor);
  } else {
    // Build failed in operational mode — still try to show whatever we got
    // so the user can see *where* the algorithm got stuck.
    const auto* failure = gPipeline.debug().firstFailure();
    viz.drawStage(failure ? failure->stage : gPipeline.lastSuccessfulStage());
  }

  gLastRouteGeo = gPipeline.state().routeGeo;
  view.mState.routePoints = gLastRouteGeo;

  restoreRouteAnchorMarkers(view, startPointParam, endPointParam, input.hasEndPoint);
}

void GeoViewRouteFeature::invalidateRouteAnchorMarkers(GeoViewWidget& view) {
  view.mState.startPointMarker = nullptr;
  view.mState.endPointMarker = nullptr;
}

void GeoViewRouteFeature::setVisualizationStage(GeoViewWidget& view,
                                                RouteAlgo::PipelineStage stage,
                                                int stepLimit) {
  if (!view.mRouteLayer || !view.mMap) return;
  invalidateRouteAnchorMarkers(view);
  RouteAlgo::RoutePipelineVisualizer viz(view.mMap, view.mRouteLayer, gPipeline.state(),
                                         gPipeline.state().stepMeters);
  viz.drawStage(stage, stepLimit);
  const bool hasEndPoint =
      view.mState.manualEndPoint.latitude() != 0.0 || view.mState.manualEndPoint.longitude() != 0.0;
  restoreRouteAnchorMarkers(view, view.mState.manualStartPoint, view.mState.manualEndPoint,
                          hasEndPoint);
}

int GeoViewRouteFeature::stageStepCount(RouteAlgo::PipelineStage stage) {
  RouteAlgo::RoutePipelineVisualizer viz(nullptr, nullptr, gPipeline.state(),
                                         gPipeline.state().stepMeters);
  return viz.stageStepCount(stage);
}

const std::vector<RouteAlgo::StageInfo>& GeoViewRouteFeature::lastPipelineStages() {
  return gPipeline.debug().stages();
}

QString GeoViewRouteFeature::lastBuildStatus() { return gLastBuildStatus; }
bool GeoViewRouteFeature::lastBuildSuccessful() { return gLastBuildOk; }

int GeoViewRouteFeature::lastPrepareOuterContourCount() {
  const RouteAlgo::StageInfo* prep =
      gPipeline.debug().find(RouteAlgo::PipelineStage::Prepare);
  if (!prep) return 1;
  for (const auto& kv : prep->metrics) {
    if (kv.first == QStringLiteral("outerContours")) {
      bool ok = false;
      const int v = kv.second.toInt(&ok);
      return ok ? std::max(1, v) : 1;
    }
  }
  return 1;
}

int GeoViewRouteFeature::lastUnreachableIslandCount() {
  return static_cast<int>(gPipeline.state().stitchResult.unreachableIslandIndices.size());
}

RouteAlgo::PipelineStage GeoViewRouteFeature::lastSuccessfulStage() {
  return gPipeline.lastSuccessfulStage();
}
const QVector<QGV::GeoPos>& GeoViewRouteFeature::lastRouteGeo() { return gLastRouteGeo; }

QVector<QGV::GeoPos> GeoViewRouteFeature::buildRouteWithAngle(GeoViewWidget& view,
                                                              double stepMeters,
                                                              double angleDegrees,
                                                              double offsetFromContour,
                                                              double offsetCut) {
  Q_UNUSED(offsetCut);
  if (view.mContour->points().size() < 2 || !view.mMap) return {};

  QVector<QPointF> projContour;
  projContour.reserve(view.mContour->points().size());
  for (const auto& g : view.mContour->points()) {
    projContour.push_back(view.mMap->getProjection()->geoToProj(g));
  }

  QPointF startProj = projContour.front();
  if (view.mState.manualStartPoint.latitude() != 0.0 ||
      view.mState.manualStartPoint.longitude() != 0.0) {
    startProj = view.mMap->getProjection()->geoToProj(view.mState.manualStartPoint);
  } else if (view.mRobotItem.item) {
    startProj = view.mMap->getProjection()->geoToProj(view.mRobotItem.pos);
  }

  constexpr bool kDefaultRightSide = true;
  const QVector<QPointF> projRoute = buildBasicCoverageProj(
      projContour, stepMeters, angleDegrees, offsetFromContour, startProj, kDefaultRightSide,
      true);
  if (projRoute.size() < 2) return {};

  QVector<QGV::GeoPos> snakePath;
  snakePath.reserve(projRoute.size());
  for (const auto& p : projRoute) {
    snakePath.push_back(view.mMap->getProjection()->projToGeo(p));
  }

  view.clearRouteLayer();
  return snakePath;
}

bool GeoViewRouteFeature::buildBasicParallelCoverageRoute(GeoViewWidget& view, double stepMeters,
                                                          double angleDegrees,
                                                          double offsetFromContour,
                                                          double offsetCut,
                                                          const QGV::GeoPos& routeStartPoint,
                                                          const QGV::GeoPos& routeEndPoint,
                                                          bool hasEndPoint, bool rightSide) {
  Q_UNUSED(offsetCut);
  gLastBuildOk = false;
  gLastBuildStatus = QObject::tr("Маршрут не построен");
  gLastRouteGeo.clear();

  if (view.mContour->points().size() < 2 || !view.mRouteLayer || !view.mMap) return false;

  QVector<QPointF> projContour;
  projContour.reserve(view.mContour->points().size());
  for (const auto& g : view.mContour->points()) {
    projContour.push_back(view.mMap->getProjection()->geoToProj(g));
  }

  const bool hasStartPoint =
      routeStartPoint.latitude() != 0.0 || routeStartPoint.longitude() != 0.0;
  QPointF startProj;
  if (hasStartPoint) {
    startProj = view.mMap->getProjection()->geoToProj(routeStartPoint);
  } else if (view.mRobotItem.item) {
    startProj = view.mMap->getProjection()->geoToProj(view.mRobotItem.pos);
  } else {
    startProj = projContour.front();
  }
  const QPointF endProj =
      hasEndPoint ? view.mMap->getProjection()->geoToProj(routeEndPoint) : startProj;

  const QVector<QPointF> fullProj = buildBasicFullRouteProj(
      projContour, stepMeters, angleDegrees, offsetFromContour, startProj, endProj, hasEndPoint,
      rightSide, true);
  if (fullProj.size() < 2) return false;

  QVector<QGV::GeoPos> fullRoute;
  fullRoute.reserve(fullProj.size());
  for (const auto& p : fullProj) {
    appendGeoPoint(fullRoute, view.mMap->getProjection()->projToGeo(p));
  }

  view.clearRouteLayer();
  invalidateRouteAnchorMarkers(view);

  auto* poly = new GeoPolyline(view.mMap);
  poly->setPen(view.mRouteColor);
  poly->points = fullRoute;
  view.mRouteLayer->addItem(poly);
  view.mState.routePoints = fullRoute;
  gLastRouteGeo = fullRoute;
  gLastBuildOk = true;
  gLastBuildStatus = QObject::tr("Упрощённый алгоритм: угол %1°, %2 точек")
                         .arg(angleDegrees)
                         .arg(fullRoute.size());
  return true;
}

void GeoViewRouteFeature::handleMapClickStartRoutePoint(GeoViewWidget& view,
                                                        const QGV::GeoPos& pos) {
  Q_UNUSED(pos);
  if (view.interactionMode() != GeoViewWidget::InteractionMode::SelectStartPoint &&
      view.interactionMode() != GeoViewWidget::InteractionMode::SelectEndPoint)
    return;

  auto addRectMarker = [&](const QGV::GeoPos& point) {
    constexpr double sizeMeters = 0.5;
    const double dLat = sizeMeters / 111000.0;
    const double dLon = sizeMeters / (111000.0 * std::cos(qDegreesToRadians(point.latitude())));
    const QGV::GeoPos p1(point.latitude() - dLat / 2, point.longitude() - dLon / 2);
    const QGV::GeoPos p2(point.latitude() + dLat / 2, point.longitude() + dLon / 2);
    auto* rectItem = new Rectangle(QGV::GeoRect(p1, p2), Qt::red);
    rectItem->setZValue(1000);
    QGVItem*& existing =
        (view.interactionMode() == GeoViewWidget::InteractionMode::SelectStartPoint)
            ? view.mState.startPointMarker
            : view.mState.endPointMarker;
    if (view.mRouteLayer) {
      if (existing) {
        view.mRouteLayer->removeItem(existing);
        delete existing;
        existing = nullptr;
      }
      view.mRouteLayer->addItem(rectItem);
      existing = rectItem;
      view.mRouteLayer->update();
    }
  };

  if (view.mState.manualStartPoint.latitude() != 0 || view.mState.manualStartPoint.longitude() != 0) {
    addRectMarker(view.mState.manualStartPoint);
    view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
  }
  if (view.mState.manualEndPoint.latitude() != 0 || view.mState.manualEndPoint.longitude() != 0) {
    addRectMarker(view.mState.manualEndPoint);
    view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
  }
}

void GeoViewRouteFeature::handleMapClick(GeoViewWidget& view, const QGV::GeoPos& pos) {
  auto* item = new QGVIcon();
  item->setGeometry(pos);
  item->loadImage(view.mRobotIcon);
  view.mRouteLayer->addItem(item);
  view.mState.routePoints.append(pos);
  drawRoute(view, view.mRouteLayer, view.mState.routePoints, view.mRouteColor, true, false);
}

void GeoViewRouteFeature::drawRoute(GeoViewWidget& view, QGVLayer* layer,
                                    const QVector<QGV::GeoPos>& pts, const QPen& pen,
                                    bool drawArrow, bool replaceExisting) {
  if (!view.mRouteLayer) return;
  if (!replaceExisting) layer->deleteItems();
  if (pts.size() < 2) return;
  auto* poly = new GeoPolyline(view.mMap);
  poly->points = pts;
  poly->setPen(pen);
  poly->drawArrowOnEnd = drawArrow;
  layer->addItem(poly);
}

void GeoViewRouteFeature::addRouteAnchorMarker(GeoViewWidget& view, const QGV::GeoPos& point,
                                             const QColor& color, QGVItem*& markerSlot) {
  if (!view.mRouteLayer || !view.mMap) return;
  markerSlot = nullptr;
  constexpr double sizeMeters = 0.8;
  const double dLat = sizeMeters / 111000.0;
  const double dLon = sizeMeters / (111000.0 * std::cos(qDegreesToRadians(point.latitude())));
  const QGV::GeoPos p1(point.latitude() - dLat / 2, point.longitude() - dLon / 2);
  const QGV::GeoPos p2(point.latitude() + dLat / 2, point.longitude() + dLon / 2);
  auto* rectItem = new Rectangle(QGV::GeoRect(p1, p2), color);
  rectItem->setZValue(1200);
  view.mRouteLayer->addItem(rectItem);
  markerSlot = rectItem;
}

void GeoViewRouteFeature::restoreRouteAnchorMarkers(GeoViewWidget& view,
                                                  const QGV::GeoPos& startPoint,
                                                  const QGV::GeoPos& endPoint, bool hasEndPoint) {
  addRouteAnchorMarker(view, startPoint, QColor(255, 215, 0, 230), view.mState.startPointMarker);
  if (hasEndPoint) {
    addRouteAnchorMarker(view, endPoint, QColor(255, 120, 80, 220), view.mState.endPointMarker);
  }
}

void GeoViewRouteFeature::toggleManualRouteMode(GeoViewWidget& view) {
  const bool enableManual =
      (view.interactionMode() != GeoViewWidget::InteractionMode::ManualRoute);
  if (enableManual) {
    view.setInteractionMode(GeoViewWidget::InteractionMode::ManualRoute);
    view.mState.routePoints.clear();
    if (view.mRobotItem.item) view.mState.routePoints.prepend(view.mRobotItem.pos);
    if (!view.mRouteLayer) {
      view.mRouteLayer = new QGVLayer();
      view.mMap->addItem(view.mRouteLayer);
    }
    view.mRouteLayer->deleteItems();
    view.setUiStatus(view.tr("Ручной режим: кликайте по карте для добавления точек"),
                     GeoViewWidget::UiStatusLevel::Info);
  } else {
    view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
    view.setUiStatus(view.tr("Ручной маршрут завершен: %1 точек").arg(view.mState.routePoints.size()),
                     GeoViewWidget::UiStatusLevel::Success);
  }
}
