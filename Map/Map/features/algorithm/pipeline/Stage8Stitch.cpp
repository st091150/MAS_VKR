#include "Stage8Stitch.h"

#include "RoutePipelineDebug.h"

namespace RouteAlgo {

bool runStageStitch(const RoutePipelineInput& input, RoutePipelineState& state,
                    RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Stitch);

  if (!state.connectorPolicy) {
    debug.fail(QStringLiteral("Не инициализирована connector policy"));
    return false;
  }

  StitchParams sp;
  sp.nearestPt = state.nearestPt;
  sp.lineDir = state.lineDir;
  sp.normal = state.normal;
  sp.stepMeters = input.stepMeters;
  sp.componentCount = static_cast<int>(state.components.size());
  sp.intraStrict = state.tuning.intraStrict;
  sp.invalidConnectorCost = state.tuning.invalidConnectorCost;
  sp.routeDebug = input.routeDebug;
  sp.config = input.config;
  sp.connectorCost = [&](const QPointF& from, const QPointF& to) {
    return state.connectorPolicy->connectorCost(from, to, ConnectorHopKind::InterIsland);
  };
  sp.appendConnector = [&](QVector<QPointF>& projRoute, const QPointF& from, const QPointF& to) {
    return state.connectorPolicy->appendConnectorProj(projRoute, from, to,
                                                      ConnectorHopKind::InterIsland);
  };

  state.stitchResult = stitchIslands(state.islands, sp);
  state.stitchedProj = state.stitchResult.stitchedProj;
  state.islandLinks = state.stitchResult.islandLinks;

  int merged = 0;
  for (const auto& l : state.islandLinks) {
    if (l.merged) ++merged;
  }
  debug.metric(QStringLiteral("usedIslands"), state.stitchResult.usedIslandCount);
  debug.metric(QStringLiteral("totalIslands"), static_cast<int>(state.islands.size()));
  debug.metric(QStringLiteral("unreachableIslands"),
               static_cast<int>(state.stitchResult.unreachableIslandIndices.size()));
  debug.metric(QStringLiteral("transitions"), static_cast<int>(state.islandLinks.size()));
  debug.metric(QStringLiteral("mergedTransitions"), merged);
  debug.metric(QStringLiteral("stitchedPoints"),
               static_cast<int>(state.stitchedProj.size()));
  if (!state.stitchResult.unreachableIslandIndices.empty()) {
    for (int islandIdx : state.stitchResult.unreachableIslandIndices) {
      if (islandIdx < 0 || islandIdx >= static_cast<int>(state.islands.size())) continue;
      const auto& island = state.islands[static_cast<size_t>(islandIdx)];
      debug.log(QStringLiteral("Недостижимый остров id=%1 comp=%2")
                    .arg(island.id)
                    .arg(island.compId));
    }
    if (state.stitchedProj.size() < 2) {
      debug.fail(QStringLiteral("Есть недостижимые острова: %1. Недостаточно точек по достижимым участкам.")
                     .arg(static_cast<int>(state.stitchResult.unreachableIslandIndices.size())));
      return false;
    }
    debug.summarize(QStringLiteral("Маршрут по %1 из %2 островов; недостижимых: %3")
                        .arg(state.stitchResult.usedIslandCount)
                        .arg(static_cast<int>(state.islands.size()))
                        .arg(static_cast<int>(state.stitchResult.unreachableIslandIndices.size())));
    debug.endStage(StageStatus::Ok);
    return true;
  }
  if (state.stitchedProj.size() < 2) {
    debug.fail(QStringLiteral("Сшивка не дала достаточного количества точек"));
    return false;
  }
  debug.summarize(QStringLiteral("Сшито %1 островов через %2 переходов")
                      .arg(state.stitchResult.usedIslandCount)
                      .arg(static_cast<int>(state.islandLinks.size())));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
