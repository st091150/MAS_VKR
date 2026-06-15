#include "Stage9Approach.h"

#include "RoutePipelineDebug.h"

#include "../RouteConnectorPolicy.h"

#include <QGeoView/QGVMap.h>

#include <cmath>
#include <limits>

namespace RouteAlgo {

namespace {
// Two consecutive points closer than this are treated as duplicates and
// collapsed when assembling the final polyline.
constexpr double kDuplicateMergeMeters = 0.02;

QVector<QPointF> buildApproachLeg(const RegionGeometry& region, const RouteAlgorithmConfig& config,
                                  double stepMeters, double invalidConnectorCost, bool routeDebug,
                                  const QPointF& fromPoint, const QPointF& toPoint) {
  RouteConnectorPolicy policy(region, config, stepMeters, invalidConnectorCost, routeDebug);
  QVector<QPointF> leg;
  leg.push_back(fromPoint);
  if (!policy.appendConnectorProj(leg, fromPoint, toPoint, ConnectorHopKind::InterIsland)) {
    return {};
  }
  return leg;
}
}  // namespace

bool runStageApproach(const RoutePipelineInput& input, RoutePipelineState& state,
                      RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Approach);

  if (state.stitchedProj.size() < 2) {
    debug.fail(QStringLiteral("Нет сшитого маршрута для подвода"));
    return false;
  }

  const RegionGeometry& legRegion = state.insetGeo;

  // Build approach: start anchor → first stitched point along the outer boundary
  // (contour − cutouts), not a straight chord through forbidden zones.
  const QPointF firstPt = state.stitchedProj.front();
  state.approachProj =
      buildApproachLeg(legRegion, input.config, state.stepMeters, state.tuning.invalidConnectorCost,
                       input.routeDebug, input.startProj, firstPt);
  if (state.approachProj.size() < 2) {
    debug.fail(QStringLiteral("Не удалось построить подводку без пересечения запретных зон"));
    return false;
  }

  QVector<QPointF> fullProj;
  fullProj.push_back(input.startProj);
  auto appendUnique = [&](const QVector<QPointF>& src, int fromIdx) {
    for (int i = fromIdx; i < src.size(); ++i) {
      if (fullProj.isEmpty() ||
          std::hypot((src[i] - fullProj.back()).x(), (src[i] - fullProj.back()).y()) >
              kDuplicateMergeMeters) {
        fullProj.push_back(src[i]);
      }
    }
  };
  appendUnique(state.approachProj, 0);
  appendUnique(state.stitchedProj, 1);

  // Return: last point → end target (or back to start when no end is set).
  const QPointF endTarget = input.hasEndPoint ? input.endProj : input.startProj;
  state.returnProj =
      buildApproachLeg(legRegion, input.config, state.stepMeters, state.tuning.invalidConnectorCost,
                       input.routeDebug, fullProj.back(), endTarget);
  if (state.returnProj.size() < 2) {
    debug.fail(QStringLiteral("Не удалось построить возврат без пересечения запретных зон"));
    return false;
  }
  appendUnique(state.returnProj, 1);

  state.fullRouteProj = std::move(fullProj);

  state.routeGeo.clear();
  state.routeGeo.reserve(state.fullRouteProj.size());
  if (input.map) {
    for (const auto& p : state.fullRouteProj) {
      state.routeGeo.push_back(input.map->getProjection()->projToGeo(p));
    }
  }

  debug.metric(QStringLiteral("approachPoints"), static_cast<int>(state.approachProj.size()));
  debug.metric(QStringLiteral("returnPoints"), static_cast<int>(state.returnProj.size()));
  debug.metric(QStringLiteral("totalPoints"), static_cast<int>(state.fullRouteProj.size()));
  debug.summarize(QStringLiteral("Маршрут собран: %1 точек, заход %2 / возврат %3")
                      .arg(static_cast<int>(state.fullRouteProj.size()))
                      .arg(static_cast<int>(state.approachProj.size()))
                      .arg(static_cast<int>(state.returnProj.size())));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
