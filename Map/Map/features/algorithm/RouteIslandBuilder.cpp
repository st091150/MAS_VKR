#include "RouteIslandBuilder.h"

#include "../../utils/MapLog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>

namespace RouteAlgo {
namespace {
constexpr double kGeomEps = 1e-6;
// Hard floor on the geometry gate slack so that on extremely small steps the
// gate doesn't collapse below numerical noise.
constexpr double kMinGeometryGateSlackMeters = 0.5;
// Two stripe segments on the *same* sweep row belong to different physical
// "pieces" of that line if their along-line intervals are separated by more
// than this gap (meters). They must not share an island — there is contour /
// empty space between them (concave polygon). Clustering enforces that before
// we even look at row-to-row edges.
constexpr double kSameRowClusterAlongGapMeters = 0.05;

bool rowEdgeConnectsInsideRegion(const StripeSegment& sa, const StripeSegment& sb,
                                 const RegionGeometry& region) {
  if (!regionContainsPoint(region, sa.mid) || !regionContainsPoint(region, sb.mid)) {
    return false;
  }
  const QPointF endsA[2] = {sa.a, sa.b};
  const QPointF endsB[2] = {sb.a, sb.b};
  for (const QPointF& pa : endsA) {
    for (const QPointF& pb : endsB) {
      if (directConnectorInsideRegion(region, pa, pb)) return true;
    }
  }
  return directConnectorInsideRegion(region, sa.mid, sb.mid);
}
}  // namespace

IslandGraphBuildResult buildIslandGraph(const std::vector<std::vector<StripeSegment>>& rowsSegments,
                                        int rowCount, const QPointF& lineDir, const QPointF& normal,
                                        double stepMeters, const RouteAlgorithmConfig& config,
                                        const RegionGeometry& insetGeo,
                                        const RegionGeometry& workingGeo, bool routeDebug) {
  IslandGraphBuildResult out;
  auto nodeTouchesExclusion = [&](int nodeId) {
    if (nodeId < 0 || nodeId >= static_cast<int>(out.nodes.size())) return false;
    return stripeTouchesExclusionBoundary(insetGeo, workingGeo,
                                          out.nodes[static_cast<size_t>(nodeId)].seg, stepMeters);
  };
  std::vector<std::vector<int>> rowNodeIds(static_cast<size_t>(rowCount));
  for (int rowIdx = 0; rowIdx < rowCount; ++rowIdx) {
    const auto& rowSegs = rowsSegments[static_cast<size_t>(rowIdx)];
    auto& ids = rowNodeIds[static_cast<size_t>(rowIdx)];
    ids.reserve(rowSegs.size());
    for (const auto& seg : rowSegs) {
      IslandGraphNode n;
      n.seg = seg;
      n.id = static_cast<int>(out.nodes.size());
      out.nodes.push_back(std::move(n));
      ids.push_back(out.nodes.back().id);
    }
  }

  auto segRangeOnLine = [&](const StripeSegment& s) {
    const double ta = QPointF::dotProduct(s.a, lineDir);
    const double tb = QPointF::dotProduct(s.b, lineDir);
    return std::pair<double, double>(std::min(ta, tb), std::max(ta, tb));
  };
  auto segDirection = [](const StripeSegment& s) { return normalizeOrZero(s.b - s.a); };
  auto containsId = [](const std::vector<int>& v, int id) {
    return std::find(v.begin(), v.end(), id) != v.end();
  };

  struct EdgeCandidate {
    int toId = -1;
    double score = std::numeric_limits<double>::max();
  };

  // Single-pass: only row+1 strict edges with one geometry gate.
  // No soft edges, no row+2 fallback, no adaptive density scaling.
  const double slack =
      std::max(kMinGeometryGateSlackMeters, stepMeters * config.geometryGateSlackFactor);

  // --- Along-line clusters on each sweep row: disjoint stripe intervals on
  // the same row never share an island (concave polygon "teeth").
  // Also split by `StripeSegment::pathIndex` so disjoint inset outers (separate
  // Clipper outer rings after cutouts) never share a cluster on one row — they
  // used to merge when along-gaps were tiny and then row edges could bridge
  // across empty map space between real islands.
  std::vector<int> nodeGlobalCluster(static_cast<size_t>(out.nodes.size()), -1);
  int globalClusterCount = 0;
  for (int rowIdx = 0; rowIdx < rowCount; ++rowIdx) {
    const auto& ids = rowNodeIds[static_cast<size_t>(rowIdx)];
    if (ids.empty()) continue;
    std::map<int, std::vector<int>> byPathIndex;
    for (int id : ids) {
      byPathIndex[out.nodes[static_cast<size_t>(id)].seg.pathIndex].push_back(id);
    }
    for (auto& kv : byPathIndex) {
      auto& pathIds = kv.second;
      std::sort(pathIds.begin(), pathIds.end(), [&](int id1, int id2) {
        const auto r1 = segRangeOnLine(out.nodes[static_cast<size_t>(id1)].seg);
        const auto r2 = segRangeOnLine(out.nodes[static_cast<size_t>(id2)].seg);
        if (r1.first != r2.first) return r1.first < r2.first;
        return r1.second < r2.second;
      });
      double clusterMaxAlong = -std::numeric_limits<double>::infinity();
      bool clusterTouchesExclusion = false;
      for (int id : pathIds) {
        const auto r = segRangeOnLine(out.nodes[static_cast<size_t>(id)].seg);
        const bool touchesExclusion = nodeTouchesExclusion(id);
        if (clusterMaxAlong < -1e50 || r.first > clusterMaxAlong + kSameRowClusterAlongGapMeters ||
            touchesExclusion != clusterTouchesExclusion) {
          ++globalClusterCount;
          clusterTouchesExclusion = touchesExclusion;
        }
        nodeGlobalCluster[static_cast<size_t>(id)] = globalClusterCount - 1;
        clusterMaxAlong = std::max(clusterMaxAlong, r.second);
      }
    }
  }

  std::vector<std::vector<int>> clusterAdj(static_cast<size_t>(globalClusterCount));
  std::vector<bool> clusterExclusionTouch(static_cast<size_t>(globalClusterCount), false);
  for (int nid = 0; nid < static_cast<int>(out.nodes.size()); ++nid) {
    const int gc = nodeGlobalCluster[static_cast<size_t>(nid)];
    if (gc < 0) continue;
    if (nodeTouchesExclusion(nid)) {
      clusterExclusionTouch[static_cast<size_t>(gc)] = true;
    }
  }
  auto addClusterEdge = [&](int a, int b) {
    if (a < 0 || b < 0 || a == b) return;
    if (clusterExclusionTouch[static_cast<size_t>(a)] !=
        clusterExclusionTouch[static_cast<size_t>(b)]) {
      return;
    }
    auto& va = clusterAdj[static_cast<size_t>(a)];
    if (std::find(va.begin(), va.end(), b) == va.end()) va.push_back(b);
    auto& vb = clusterAdj[static_cast<size_t>(b)];
    if (std::find(vb.begin(), vb.end(), a) == vb.end()) vb.push_back(a);
  };

  for (int rowIdx = 0; rowIdx + 1 < rowCount; ++rowIdx) {
    const auto& ra = rowNodeIds[static_cast<size_t>(rowIdx)];
    const auto& rb = rowNodeIds[static_cast<size_t>(rowIdx + 1)];
    if (ra.empty() || rb.empty()) {
      continue;
    }
    for (int ia : ra) {
      const StripeSegment& sa = out.nodes[static_cast<size_t>(ia)].seg;
      const auto rangeA = segRangeOnLine(sa);
      const QPointF dirA = segDirection(sa);

      std::vector<EdgeCandidate> candidates;
      candidates.reserve(rb.size());
      for (int ib : rb) {
        const StripeSegment& sb = out.nodes[static_cast<size_t>(ib)].seg;
        if (sa.pathIndex != sb.pathIndex) continue;
        const auto rangeB = segRangeOnLine(sb);
        const double overlap = std::min(rangeA.second, rangeB.second) -
                               std::max(rangeA.first, rangeB.first);
        const QPointF delta = sb.mid - sa.mid;
        const double alongAbs = std::abs(QPointF::dotProduct(delta, lineDir));
        const double acrossAbs = std::abs(QPointF::dotProduct(delta, normal));
        const double acrossMismatch = std::abs(acrossAbs - stepMeters);
        const QPointF dirB = segDirection(sb);
        const double dirDot = std::abs(QPointF::dotProduct(dirA, dirB));
        const double angleDiff = 1.0 - std::clamp(dirDot, 0.0, 1.0);
        const double gapAlong = alongAbs / std::max(stepMeters, kGeomEps);
        const double gapAcross = acrossMismatch / std::max(stepMeters, kGeomEps);
        const double score = config.scoreWeightAlong * gapAlong +
                             config.scoreWeightAcross * gapAcross +
                             config.scoreWeightAngle * angleDiff;
        const bool geometryGate = (overlap >= -slack || alongAbs <= slack) &&
                                  (acrossMismatch <= slack);
        if (!geometryGate) {
          continue;
        }
        if (score > config.scoreStrictThreshold) {
          continue;
        }
        if (nodeTouchesExclusion(ia) != nodeTouchesExclusion(ib)) {
          continue;
        }
        if (!rowEdgeConnectsInsideRegion(sa, sb, insetGeo)) {
          continue;
        }
        candidates.push_back({ib, score});
      }
      if (candidates.empty()) {
        continue;
      }
      std::sort(candidates.begin(), candidates.end(),
                [](const EdgeCandidate& lhs, const EdgeCandidate& rhs) { return lhs.score < rhs.score; });
      const int kCap = std::max(1, config.topKNeighbors);
      const int take = std::min(kCap, static_cast<int>(candidates.size()));
      for (int k = 0; k < take; ++k) {
        const int ib = candidates[static_cast<size_t>(k)].toId;
        if (!containsId(out.nodes[static_cast<size_t>(ia)].adjStrict, ib)) {
          out.nodes[static_cast<size_t>(ia)].adjStrict.push_back(ib);
        }
        if (!containsId(out.nodes[static_cast<size_t>(ib)].adjStrict, ia)) {
          out.nodes[static_cast<size_t>(ib)].adjStrict.push_back(ia);
        }
        const int ca = nodeGlobalCluster[static_cast<size_t>(ia)];
        const int cb = nodeGlobalCluster[static_cast<size_t>(ib)];
        addClusterEdge(ca, cb);
      }
    }
  }

  if (routeDebug) {
    int strictEdges = 0;
    for (const auto& n : out.nodes) {
      strictEdges += static_cast<int>(n.adjStrict.size());
    }
    qCDebug(logPipeline) << "[route-graph]"
             << "nodes=" << out.nodes.size()
             << "edges=" << (strictEdges / 2)
             << "alongLineClusters=" << globalClusterCount
             << "exclusionClusters="
             << std::count(clusterExclusionTouch.begin(), clusterExclusionTouch.end(), true)
             << "topK=" << config.topKNeighbors;
  }

  std::vector<std::vector<int>> nodesPerCluster(static_cast<size_t>(globalClusterCount));
  for (int nid = 0; nid < static_cast<int>(out.nodes.size()); ++nid) {
    const int gc = nodeGlobalCluster[static_cast<size_t>(nid)];
    if (gc >= 0) {
      nodesPerCluster[static_cast<size_t>(gc)].push_back(nid);
    }
  }

  std::vector<bool> clusterVisited(static_cast<size_t>(globalClusterCount), false);
  for (int c = 0; c < globalClusterCount; ++c) {
    if (clusterVisited[static_cast<size_t>(c)]) continue;
    std::vector<int> cstack{c};
    clusterVisited[static_cast<size_t>(c)] = true;
    std::vector<int> clusterComp;
    while (!cstack.empty()) {
      const int u = cstack.back();
      cstack.pop_back();
      clusterComp.push_back(u);
      for (int v : clusterAdj[static_cast<size_t>(u)]) {
        if (v < 0 || v >= globalClusterCount || clusterVisited[static_cast<size_t>(v)]) continue;
        clusterVisited[static_cast<size_t>(v)] = true;
        cstack.push_back(v);
      }
    }
    std::vector<int> nodeComp;
    for (int gc : clusterComp) {
      const auto& bucket = nodesPerCluster[static_cast<size_t>(gc)];
      nodeComp.insert(nodeComp.end(), bucket.begin(), bucket.end());
    }
    if (!nodeComp.empty()) {
      out.components.push_back(std::move(nodeComp));
    }
  }

  return out;
}

}  // namespace RouteAlgo
