#include "Stage6Routes.h"

#include "RoutePipelineDebug.h"

#include "../RouteConnectorPolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace RouteAlgo {

namespace {
// Distance used to push the "virtual cursor" before the first segment in a
// row group. Any value much larger than the polygon diagonal makes the row's
// far end unambiguously the entry point — the snake direction picks the side.
constexpr double kVirtualCursorOffsetMeters = 1e6;
constexpr double kMaxBoundaryDetourRatio = 2.5;
constexpr double kBoundaryDetourMinStepFactor = 2.5;

double euclid(const QPointF& a, const QPointF& b) {
  return std::hypot((b - a).x(), (b - a).y());
}

std::pair<QPointF, QPointF> stripeOrientedEnds(const StripeSegment& seg, bool forwardAlongLine,
                                               const QPointF& lineDir) {
  const double ta = QPointF::dotProduct(seg.a, lineDir);
  const double tb = QPointF::dotProduct(seg.b, lineDir);
  if (forwardAlongLine) {
    return (ta <= tb) ? std::pair<QPointF, QPointF>(seg.a, seg.b)
                      : std::pair<QPointF, QPointF>(seg.b, seg.a);
  }
  return (ta <= tb) ? std::pair<QPointF, QPointF>(seg.b, seg.a)
                    : std::pair<QPointF, QPointF>(seg.a, seg.b);
}
}  // namespace

bool runStageRoutes(const RoutePipelineInput& input, RoutePipelineState& state,
                    RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Routes);

  if (!state.connectorPolicy) {
    debug.fail(QStringLiteral("Не инициализирована connector policy"));
    return false;
  }
  auto connectorCost = [&](const QPointF& from, const QPointF& to) {
    return state.connectorPolicy->connectorCost(from, to, ConnectorHopKind::IntraIslandSnake);
  };
  auto appendConnectorProj = [&](QVector<QPointF>& projRoute, const QPointF& from,
                                 const QPointF& to) {
    return state.connectorPolicy->appendConnectorProj(projRoute, from, to,
                                                      ConnectorHopKind::IntraIslandSnake);
  };

  std::vector<RoutedChunk> routedChunks;
  routedChunks.reserve(state.components.size() * 2);
  int acceptedSegments = 0;
  int splitChunks = 0;

  for (int compIdx = 0; compIdx < static_cast<int>(state.components.size()); ++compIdx) {
    const auto& comp = state.components[static_cast<size_t>(compIdx)];
    if (comp.empty()) continue;
    acceptedSegments += static_cast<int>(comp.size());

    std::vector<std::pair<int, int>> rowAndNode;
    rowAndNode.reserve(comp.size());
    for (int nodeId : comp) {
      rowAndNode.emplace_back(state.nodes[static_cast<size_t>(nodeId)].seg.row, nodeId);
    }
    std::sort(rowAndNode.begin(), rowAndNode.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (int phase = 0; phase < 2; ++phase) {
      QVector<QPointF> projRoute;
      int chunkMinRow = std::numeric_limits<int>::max();
      int chunkMaxRow = std::numeric_limits<int>::min();
      auto flushChunk = [&]() {
        if (projRoute.size() <= 1) return;
        RoutedChunk rc;
        rc.pts = projRoute;
        rc.compId = compIdx;
        rc.phase = phase;
        rc.minRow = chunkMinRow;
        rc.maxRow = chunkMaxRow;
        routedChunks.push_back(std::move(rc));
      };
      auto resetChunkRows = [&](int rowId) {
        chunkMinRow = rowId;
        chunkMaxRow = rowId;
      };
      auto noteRow = [&](int rowId) {
        chunkMinRow = std::min(chunkMinRow, rowId);
        chunkMaxRow = std::max(chunkMaxRow, rowId);
      };

      bool forwardAlongLine =
          (phase == 0) ? input.forwardAlongLineFirst : !input.forwardAlongLineFirst;
      int i = 0;
      while (i < static_cast<int>(rowAndNode.size())) {
        const int rowId = rowAndNode[static_cast<size_t>(i)].first;
        int j = i;
        while (j < static_cast<int>(rowAndNode.size()) &&
               rowAndNode[static_cast<size_t>(j)].first == rowId) {
          ++j;
        }
        std::vector<int> rowNodes;
        rowNodes.reserve(static_cast<size_t>(j - i));
        for (int k = i; k < j; ++k) {
          rowNodes.push_back(rowAndNode[static_cast<size_t>(k)].second);
        }

        std::vector<int> unvisited = rowNodes;
        QPointF cursorPoint = projRoute.isEmpty() ? QPointF() : projRoute.back();
        while (!unvisited.empty()) {
          int bestAt = -1;
          QPointF bestStart;
          QPointF bestEnd;
          double bestCost = std::numeric_limits<double>::max();
          if (projRoute.isEmpty()) {
            const auto& firstSeg = state.nodes[static_cast<size_t>(unvisited.front())].seg;
            cursorPoint = firstSeg.mid + (forwardAlongLine ? -state.lineDir : state.lineDir) *
                                             kVirtualCursorOffsetMeters;
          }
          for (int idx = 0; idx < static_cast<int>(unvisited.size()); ++idx) {
            const int nodeId = unvisited[static_cast<size_t>(idx)];
            const auto& seg = state.nodes[static_cast<size_t>(nodeId)].seg;
            const auto ends = stripeOrientedEnds(seg, forwardAlongLine, state.lineDir);
            const QPointF& segStart = ends.first;
            const QPointF& segEnd = ends.second;
            if (!projRoute.isEmpty()) {
              const double intraCost = connectorCost(projRoute.back(), segStart);
              if (!std::isfinite(intraCost) || intraCost >= state.tuning.invalidConnectorCost) {
                continue;
              }
            }
            const double c = projRoute.isEmpty()
                                 ? (forwardAlongLine
                                        ? QPointF::dotProduct(segStart, state.lineDir)
                                        : -QPointF::dotProduct(segStart, state.lineDir))
                                 : euclid(cursorPoint, segStart);
            if (c < bestCost) {
              bestCost = c;
              bestAt = idx;
              bestStart = segStart;
              bestEnd = segEnd;
            }
          }
          if (bestAt < 0 && !unvisited.empty()) {
            if (projRoute.size() > 1) {
              flushChunk();
              ++splitChunks;
              projRoute.clear();
              resetChunkRows(rowId);
            }
            for (int idx = 0; idx < static_cast<int>(unvisited.size()); ++idx) {
              const int nodeId = unvisited[static_cast<size_t>(idx)];
              const auto& seg = state.nodes[static_cast<size_t>(nodeId)].seg;
              const auto ends = stripeOrientedEnds(seg, forwardAlongLine, state.lineDir);
              const double c = projRoute.isEmpty()
                                   ? (forwardAlongLine
                                          ? QPointF::dotProduct(ends.first, state.lineDir)
                                          : -QPointF::dotProduct(ends.first, state.lineDir))
                                   : euclid(cursorPoint, ends.first);
              if (c < bestCost) {
                bestCost = c;
                bestAt = idx;
                bestStart = ends.first;
                bestEnd = ends.second;
              }
            }
          }
          if (bestAt < 0) break;
          unvisited.erase(unvisited.begin() + bestAt);
          if (projRoute.isEmpty()) {
            resetChunkRows(rowId);
            projRoute.push_back(bestStart);
            projRoute.push_back(bestEnd);
            noteRow(rowId);
          } else {
            const double intraCost = connectorCost(projRoute.back(), bestStart);
            const double directLen = euclid(projRoute.back(), bestStart);
            const double detourLimit =
                std::max(state.tuning.intraStrict,
                         std::max(directLen * kMaxBoundaryDetourRatio,
                                  input.stepMeters * kBoundaryDetourMinStepFactor));
            const bool splitIntra = !std::isfinite(intraCost) ||
                                    intraCost >= state.tuning.invalidConnectorCost ||
                                    intraCost > detourLimit;
            if (splitIntra && projRoute.size() > 1) {
              flushChunk();
              ++splitChunks;
              projRoute.clear();
              resetChunkRows(rowId);
              projRoute.push_back(bestStart);
            } else {
              const bool ok = appendConnectorProj(projRoute, projRoute.back(), bestStart);
              if (!ok) {
                flushChunk();
                ++splitChunks;
                projRoute.clear();
                resetChunkRows(rowId);
                projRoute.push_back(bestStart);
              }
            }
            projRoute.push_back(bestEnd);
            noteRow(rowId);
          }
          cursorPoint = projRoute.back();
        }

        forwardAlongLine = !forwardAlongLine;
        i = j;
      }
      flushChunk();
    }
  }

  state.routedChunks = std::move(routedChunks);
  state.acceptedSegments = acceptedSegments;

  debug.metric(QStringLiteral("acceptedSegments"), acceptedSegments);
  debug.metric(QStringLiteral("chunks"), static_cast<int>(state.routedChunks.size()));
  debug.metric(QStringLiteral("splitChunks"), splitChunks);
  if (state.routedChunks.empty()) {
    debug.fail(QStringLiteral("Не удалось сформировать покрывающие острова"));
    return false;
  }
  debug.summarize(
      QStringLiteral("Построено %1 цепочек по %2 компонентам (%3 разрывов)")
          .arg(static_cast<int>(state.routedChunks.size()))
          .arg(static_cast<int>(state.components.size()))
          .arg(splitChunks));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
