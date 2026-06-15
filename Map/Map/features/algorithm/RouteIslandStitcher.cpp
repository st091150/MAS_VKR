#include "RouteIslandStitcher.h"

#include "../../utils/MapLog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace RouteAlgo {

namespace {
constexpr double kLookaheadWeight = 0.45;
constexpr int kTinyIslandPointCount = 6;
constexpr double kTinyIslandPenaltyMinMeters = 0.8;
constexpr double kTinyIslandPenaltyStepFactor = 0.7;
constexpr double kLongJumpReferenceStepFactor = 2.0;
constexpr double kShortDirectConnectorMinMeters = 0.5;
}  // namespace

StitchResult stitchIslands(const std::vector<IslandRoute>& islands, const StitchParams& p) {
  StitchResult out;
  if (islands.empty() || !p.connectorCost || !p.appendConnector) {
    return out;
  }

  const double tinyIslandPenalty =
      std::max(kTinyIslandPenaltyMinMeters, p.stepMeters * kTinyIslandPenaltyStepFactor);
  const double shortDirectLimit =
      std::max(kShortDirectConnectorMinMeters, p.config.shortDirectConnectorMaxMeters);
  const double longJumpReference =
      std::max(p.stepMeters * kLongJumpReferenceStepFactor, shortDirectLimit);

  int maxCompId = -1;
  for (const auto& island : islands) {
    maxCompId = std::max(maxCompId, island.compId);
  }
  const int compSlots = std::max(1, maxCompId + 1);
  // -1 = component not started; 0/1 = chosen sweep phase (alternate phase is skipped).
  std::vector<int> compChosenPhase(static_cast<size_t>(compSlots), -1);

  out.stitchedProj.push_back(p.nearestPt);
  QPointF stitchFrom = p.nearestPt;
  std::vector<bool> usedIsland(static_cast<size_t>(islands.size()), false);

  auto islandEligible = [&](int islandIdx) {
    if (islandIdx < 0 || islandIdx >= static_cast<int>(islands.size())) return false;
    if (islands[static_cast<size_t>(islandIdx)].pts.size() < 2) return false;
    if (usedIsland[static_cast<size_t>(islandIdx)]) return false;
    const int compId = islands[static_cast<size_t>(islandIdx)].compId;
    if (compId < 0 || compId >= compSlots) return true;
    const int chosen = compChosenPhase[static_cast<size_t>(compId)];
    if (chosen < 0) return true;
    return islands[static_cast<size_t>(islandIdx)].phase == chosen;
  };

  struct EntryOption {
    bool reverse = false;
    double cost = std::numeric_limits<double>::max();
  };

  auto bestEntryOption = [&](int islandIdx, const QPointF& from) -> EntryOption {
    EntryOption opt;
    if (!islandEligible(islandIdx)) return opt;
    const IslandRoute& island = islands[static_cast<size_t>(islandIdx)];
    const double headCost = p.connectorCost(from, island.head);
    const double tailCost = p.connectorCost(from, island.tail);
    const bool headValid = headCost < p.invalidConnectorCost;
    const bool tailValid = tailCost < p.invalidConnectorCost;
    if (headValid && tailValid) {
      opt.reverse = tailCost < headCost;
      opt.cost = std::min(headCost, tailCost);
    } else if (headValid) {
      opt.reverse = false;
      opt.cost = headCost;
    } else if (tailValid) {
      opt.reverse = true;
      opt.cost = tailCost;
    }
    return opt;
  };

  auto islandExitPoint = [&](int islandIdx, bool reverse) {
    const IslandRoute& island = islands[static_cast<size_t>(islandIdx)];
    return reverse ? island.head : island.tail;
  };

  auto cheapEntryCost = [&](int islandIdx, const QPointF& from) {
    if (!islandEligible(islandIdx)) return std::numeric_limits<double>::max();
    const IslandRoute& island = islands[static_cast<size_t>(islandIdx)];
    return std::min(std::hypot((island.head - from).x(), (island.head - from).y()),
                    std::hypot((island.tail - from).x(), (island.tail - from).y()));
  };

  // Lookahead uses cheap distance only — full boundary routing here caused hangs.
  auto bestNextHopCost = [&](const QPointF& exit, int excludeIslandIdx) {
    double best = std::numeric_limits<double>::max();
    bool hasNext = false;
    std::vector<bool> compSeen(static_cast<size_t>(compSlots), false);
    for (int j = 0; j < static_cast<int>(islands.size()); ++j) {
      if (j == excludeIslandIdx) continue;
      if (!islandEligible(j)) continue;
      const int compId = islands[static_cast<size_t>(j)].compId;
      if (compId >= 0 && compId < compSlots) {
        if (compSeen[static_cast<size_t>(compId)]) continue;
        compSeen[static_cast<size_t>(compId)] = true;
        double compCost = std::numeric_limits<double>::max();
        for (int k = 0; k < static_cast<int>(islands.size()); ++k) {
          if (k == excludeIslandIdx || islands[k].compId != compId) continue;
          compCost = std::min(compCost, cheapEntryCost(k, exit));
        }
        if (std::isfinite(compCost)) {
          hasNext = true;
          best = std::min(best, compCost);
        }
      } else {
        const double c = cheapEntryCost(j, exit);
        if (std::isfinite(c)) {
          hasNext = true;
          best = std::min(best, c);
        }
      }
    }
    return hasNext ? best : 0.0;
  };

  int prevChosenIsland = -1;

  struct CandidateEval {
    int islandIdx = -1;
    bool reverse = false;
    double score = std::numeric_limits<double>::max();
    double entryCost = 0.0;
  };

  auto buildCandidate = [&](int i) -> CandidateEval {
    CandidateEval cand;
    const EntryOption entry = bestEntryOption(i, stitchFrom);
    if (!islandEligible(i) || entry.cost >= p.invalidConnectorCost) return cand;

    const QPointF exit = islandExitPoint(i, entry.reverse);
    const double nextHop = bestNextHopCost(exit, i);
    const double smallIslandPenalty =
        (islands[i].pts.size() <= kTinyIslandPointCount && entry.cost > p.intraStrict)
            ? tinyIslandPenalty
            : 0.0;
    const double longJumpPenalty = p.config.longJumpPenaltyWeight *
                                   std::max(0.0, (entry.cost - longJumpReference) /
                                                    std::max(p.stepMeters, 1e-6));
    cand.islandIdx = i;
    cand.reverse = entry.reverse;
    cand.entryCost = entry.cost;
    cand.score = entry.cost + kLookaheadWeight * nextHop + smallIslandPenalty + longJumpPenalty;
    return cand;
  };

  int pass = 0;
  while (pass < static_cast<int>(islands.size())) {
    std::vector<CandidateEval> passCandidates;
    passCandidates.reserve(islands.size());
    for (int i = 0; i < static_cast<int>(islands.size()); ++i) {
      const CandidateEval c = buildCandidate(i);
      if (c.islandIdx >= 0) passCandidates.push_back(c);
    }

    if (passCandidates.empty()) {
      if (p.routeDebug) {
        qCDebug(logPipeline) << "[route-stitch] no reachable candidates; finishing stitch with"
                             << out.usedIslandCount << "of" << islands.size() << "islands covered";
      }
      break;
    }

    std::sort(passCandidates.begin(), passCandidates.end(),
              [](const CandidateEval& a, const CandidateEval& b) { return a.score < b.score; });

    bool connectedAny = false;
    for (const CandidateEval& cand : passCandidates) {
      const int bestIdx = cand.islandIdx;
      const bool bestReverse = cand.reverse;

      QVector<QPointF> comp = islands[bestIdx].pts;
      if (bestReverse) {
        std::reverse(comp.begin(), comp.end());
      }

      const int rollbackSize = out.stitchedProj.size();
      const bool connected = p.appendConnector(out.stitchedProj, stitchFrom, comp.front());

      if (!connected) {
        if (out.stitchedProj.size() > rollbackSize) {
          out.stitchedProj.resize(rollbackSize);
        }
        if (p.routeDebug) {
          qCDebug(logPipeline) << "[route-stitch] policy refused connector, trying next candidate"
                               << "islandId=" << islands[bestIdx].id
                               << "compId=" << islands[bestIdx].compId
                               << "phase=" << islands[bestIdx].phase
                               << "reverse=" << bestReverse;
        }
        continue;
      }

      usedIsland[static_cast<size_t>(bestIdx)] = true;
      const int compId = islands[bestIdx].compId;
      if (compId >= 0 && compId < compSlots &&
          compChosenPhase[static_cast<size_t>(compId)] < 0) {
        compChosenPhase[static_cast<size_t>(compId)] = islands[bestIdx].phase;
        if (p.routeDebug) {
          qCDebug(logPipeline) << "[route-stitch] comp" << compId << "chose phase"
                               << islands[bestIdx].phase << "reverse=" << bestReverse;
        }
      }
      for (int pi = 1; pi < comp.size(); ++pi) {
        out.stitchedProj.push_back(comp[pi]);
      }
      if (prevChosenIsland >= 0) {
        IslandLink link;
        link.from = prevChosenIsland;
        link.to = bestIdx;
        link.merged = true;
        link.fromPoint = stitchFrom;
        link.toPoint = comp.front();
        out.islandLinks.push_back(link);
      }
      prevChosenIsland = bestIdx;
      stitchFrom = comp.back();
      ++out.usedIslandCount;
      out.transitionPointOffsets.push_back(out.stitchedProj.size());
      connectedAny = true;
      ++pass;
      break;
    }
    if (!connectedAny) {
      break;
    }
  }

  for (int i = 0; i < static_cast<int>(islands.size()); ++i) {
    if (usedIsland[static_cast<size_t>(i)]) continue;
    if (islands[i].pts.size() < 2) continue;
    const int compId = islands[i].compId;
    if (compId >= 0 && compId < compSlots &&
        compChosenPhase[static_cast<size_t>(compId)] >= 0 &&
        islands[i].phase != compChosenPhase[static_cast<size_t>(compId)]) {
      continue;
    }
    out.unreachableIslandIndices.push_back(i);
  }
  return out;
}

}  // namespace RouteAlgo
