#include "Stage4Graph.h"

#include "RoutePipelineDebug.h"

namespace RouteAlgo {

bool runStageGraph(const RoutePipelineInput& input, RoutePipelineState& state,
                   RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Graph);

  const IslandGraphBuildResult graphBuild = buildIslandGraph(
      state.rowsSegments, state.rowCount, state.lineDir, state.normal,
      input.stepMeters, input.config, state.insetGeo, state.workingGeo,
      input.routeDebug);
  state.nodes = graphBuild.nodes;
  state.rawComponents = graphBuild.components;
  // ВАЖНО: `rawComponents` — компоненты связности по "строгим" рёбрам соседства.
  // Их количество хорошо коррелирует с тем, сколько независимых "островов" маршрута
  // придётся потом сшивать (Stitch). Сильно большое значение чаще всего сигнализирует
  // о слишком маленьком шаге, шумных пересечениях или разрывах в регионе после Prepare.

  int totalEdges = 0;
  for (const auto& n : state.nodes) {
    totalEdges += static_cast<int>(n.adjStrict.size());
  }
  totalEdges /= 2;  // undirected; each edge appears twice in adjacency lists.

  debug.metric(QStringLiteral("nodes"), static_cast<int>(state.nodes.size()));
  debug.metric(QStringLiteral("edges"), totalEdges);
  debug.metric(QStringLiteral("rawComponents"),
               static_cast<int>(state.rawComponents.size()));
  if (state.nodes.empty() || state.rawComponents.empty()) {
    debug.fail(QStringLiteral("Не построен граф соседства полос"));
    return false;
  }
  debug.summarize(QStringLiteral("Граф: %1 узлов, %2 ребер, %3 компонент")
                      .arg(static_cast<int>(state.nodes.size()))
                      .arg(totalEdges)
                      .arg(static_cast<int>(state.rawComponents.size())));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
