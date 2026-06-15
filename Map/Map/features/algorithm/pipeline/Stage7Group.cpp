#include "Stage7Group.h"

#include "RoutePipelineDebug.h"

#include "../RouteConnectorPolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace RouteAlgo {

namespace {
constexpr double kMaxBoundaryDetourRatio = 2.5;
constexpr double kBoundaryDetourMinStepFactor = 2.5;

double euclid(const QPointF& a, const QPointF& b) {
  return std::hypot((b - a).x(), (b - a).y());
}

bool connectorIsAcceptable(const RouteConnectorPolicy& policy, const QPointF& from,
                           const QPointF& to, double stepMeters, double intraStrict,
                           double invalidCost, double& costOut) {
  costOut = policy.connectorCost(from, to, ConnectorHopKind::IntraIslandSnake);
  if (!std::isfinite(costOut) || costOut >= invalidCost) return false;
  const double directLen = euclid(from, to);
  const double detourLimit =
      std::max(intraStrict,
               std::max(directLen * kMaxBoundaryDetourRatio,
                        stepMeters * kBoundaryDetourMinStepFactor));
  return costOut <= detourLimit;
}

bool rowsAreContiguous(int minA, int maxA, int minB, int maxB) {
  if (minA < 0 || maxA < 0 || minB < 0 || maxB < 0) return true;
  return minB <= maxA + 1 && minA <= maxB + 1;
}
}  // namespace

bool runStageGroup(const RoutePipelineInput& input, RoutePipelineState& state,
                   RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Group);

  if (!state.connectorPolicy) {
    debug.fail(QStringLiteral("Не инициализирована connector policy"));
    return false;
  }
  auto appendConnectorProj = [&](QVector<QPointF>& projRoute, const QPointF& from,
                                 const QPointF& to) {
    return state.connectorPolicy->appendConnectorProj(projRoute, from, to,
                                                        ConnectorHopKind::IntraIslandSnake);
  };

  std::vector<IslandRoute> islands;
  islands.reserve(state.components.size());
  std::map<std::pair<int, int>, std::vector<int>> compPhaseToIslands;
  int boundaryBlockedJoins = 0;
  int connectorRefusedJoins = 0;
  int rowGapBlockedJoins = 0;

  for (const auto& chunk : state.routedChunks) {
    if (chunk.pts.size() < 2) continue;
    const std::pair<int, int> key{chunk.compId, chunk.phase};
    if (!compPhaseToIslands.count(key)) {
      IslandRoute island;
      island.pts = chunk.pts;
      island.head = chunk.pts.front();
      island.tail = chunk.pts.back();
      island.compId = chunk.compId;
      island.phase = chunk.phase;
      island.minRow = chunk.minRow;
      island.maxRow = chunk.maxRow;
      island.id = static_cast<int>(islands.size());
      island.chunkCount = 1;
      islands.push_back(std::move(island));
      compPhaseToIslands[key].push_back(islands.back().id);
      continue;
    }

    // Join chunks only when the connector is not an expensive boundary detour.
    // Never join across a row gap — that would wrap around another island's rows.
    struct AttachCandidate {
      int islandId = -1;
      bool reverse = false;
      double cost = std::numeric_limits<double>::max();
    };
    std::vector<AttachCandidate> attachCandidates;
    attachCandidates.reserve(compPhaseToIslands[key].size());
    for (int islandId : compPhaseToIslands[key]) {
      if (islandId < 0 || islandId >= static_cast<int>(islands.size())) continue;
      const auto& island = islands[static_cast<size_t>(islandId)];
      if (island.pts.size() < 2) continue;
      if (!rowsAreContiguous(island.minRow, island.maxRow, chunk.minRow, chunk.maxRow)) {
        continue;
      }
      const QPointF from = island.pts.back();
      double frontCost = std::numeric_limits<double>::max();
      double backCost = std::numeric_limits<double>::max();
      const bool frontOk =
          connectorIsAcceptable(*state.connectorPolicy, from, chunk.pts.front(), input.stepMeters,
                                state.tuning.intraStrict, state.tuning.invalidConnectorCost,
                                frontCost);
      const bool backOk =
          connectorIsAcceptable(*state.connectorPolicy, from, chunk.pts.back(), input.stepMeters,
                                state.tuning.intraStrict, state.tuning.invalidConnectorCost,
                                backCost);
      if (!frontOk && !backOk) continue;

      const bool reverseChunk = (!frontOk && backOk) || (frontOk && backOk && backCost < frontCost);
      attachCandidates.push_back(
          {islandId, reverseChunk, reverseChunk ? backCost : frontCost});
    }
    std::sort(attachCandidates.begin(), attachCandidates.end(),
              [](const AttachCandidate& a, const AttachCandidate& b) { return a.cost < b.cost; });
    bool attached = false;
    for (const AttachCandidate& cand : attachCandidates) {
      auto& island = islands[static_cast<size_t>(cand.islandId)];
      const QPointF from = island.pts.back();
      const QPointF to = cand.reverse ? chunk.pts.back() : chunk.pts.front();
      if (!appendConnectorProj(island.pts, from, to)) {
        ++connectorRefusedJoins;
        continue;
      }
      if (cand.reverse) {
        for (int pi = chunk.pts.size() - 2; pi >= 0; --pi) {
          island.pts.push_back(chunk.pts[pi]);
        }
      } else {
        for (int pi = 1; pi < chunk.pts.size(); ++pi) island.pts.push_back(chunk.pts[pi]);
      }
      island.tail = island.pts.back();
      island.minRow = std::min(island.minRow, chunk.minRow);
      island.maxRow = std::max(island.maxRow, chunk.maxRow);
      ++island.chunkCount;
      attached = true;
      break;
    }
    if (attached) continue;

    // No existing island can accept this chunk without a contour walk -> start a new island.
    ++boundaryBlockedJoins;
    if (!compPhaseToIslands[key].empty()) {
      bool anyContiguous = false;
      for (int islandId : compPhaseToIslands[key]) {
        if (islandId < 0 || islandId >= static_cast<int>(islands.size())) continue;
        const auto& island = islands[static_cast<size_t>(islandId)];
        if (rowsAreContiguous(island.minRow, island.maxRow, chunk.minRow, chunk.maxRow)) {
          anyContiguous = true;
          break;
        }
      }
      if (!anyContiguous) ++rowGapBlockedJoins;
    }
    IslandRoute island;
    island.pts = chunk.pts;
    island.head = chunk.pts.front();
    island.tail = chunk.pts.back();
    island.compId = chunk.compId;
    island.phase = chunk.phase;
    island.minRow = chunk.minRow;
    island.maxRow = chunk.maxRow;
    island.id = static_cast<int>(islands.size());
    island.chunkCount = 1;
    islands.push_back(std::move(island));
    compPhaseToIslands[key].push_back(islands.back().id);
  }

  // Centroid of coverage:
  // component. Using every polyline vertex (incl. long contour detours)
  // pulls the "+" into empty space or odd arcs — misleading for "island
  // center" in the UI.
  for (auto& island : islands) {
    QPointF centroid(0.0, 0.0);
    int count = 0;
    // When one merged component is split into multiple islands, centroid-by-component
    // is misleading. Use island polyline vertices.
    for (const auto& p : island.pts) centroid += p;
    count = island.pts.size();
    if (count > 0) centroid /= static_cast<double>(count);
    island.centroid = centroid;
  }

  state.islands = std::move(islands);

  debug.metric(QStringLiteral("chunks"), static_cast<int>(state.routedChunks.size()));
  debug.metric(QStringLiteral("islands"), static_cast<int>(state.islands.size()));
  debug.metric(QStringLiteral("boundaryBlockedJoins"), boundaryBlockedJoins);
  debug.metric(QStringLiteral("rowGapBlockedJoins"), rowGapBlockedJoins);
  debug.metric(QStringLiteral("connectorRefusedJoins"), connectorRefusedJoins);
  if (state.islands.empty()) {
    debug.fail(QStringLiteral("После группировки не осталось островов для сшивки"));
    return false;
  }
  int phaseZero = 0;
  for (const auto& isl : state.islands) {
    if (isl.phase == 0) ++phaseZero;
  }
  debug.summarize(
      QStringLiteral("Группировка: %1 островов (фаза 0: %2)")
          .arg(static_cast<int>(state.islands.size()))
          .arg(phaseZero));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
