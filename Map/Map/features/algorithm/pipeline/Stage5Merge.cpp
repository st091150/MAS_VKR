#include "Stage5Merge.h"

#include "RoutePipelineDebug.h"

#include <QPointF>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace RouteAlgo {

namespace {

bool validAnchor(const RegionGeometry& region, const BoundaryAnchor& anchor) {
  return anchor.pathIndex >= 0 && anchor.segIndex >= 0 &&
         anchor.pathIndex < static_cast<int>(region.paths.size()) &&
         region.paths[static_cast<size_t>(anchor.pathIndex)].size() >= 3;
}

void insertBoundaryAnchorForPoint(const RegionGeometry& region, const QPointF& q,
                                  std::vector<BoundaryAnchor>& out) {
  const BoundaryAnchor ba = nearestBoundaryAnchorOnRegion(region, q);
  if (!validAnchor(region, ba)) return;
  out.push_back(ba);
}

int forwardCornerCount(const BoundaryAnchor& from, const BoundaryAnchor& to, int pathN) {
  if (pathN < 3 || from.segIndex == to.segIndex) return 0;
  int count = (to.segIndex - from.segIndex + pathN) % pathN;
  constexpr double kVertexTEps = 0.04;
  if (from.t >= 1.0 - kVertexTEps && count > 0)
    --count;
  if (to.t <= kVertexTEps && count > 0)
    --count;
  return count;
}

int shortestCornerCount(const RegionGeometry& region, const BoundaryAnchor& a,
                        const BoundaryAnchor& b) {
  if (!validAnchor(region, a) || !validAnchor(region, b)) return std::numeric_limits<int>::max();
  if (a.pathIndex != b.pathIndex) return std::numeric_limits<int>::max();
  const int pathN = static_cast<int>(region.paths[static_cast<size_t>(a.pathIndex)].size());
  return std::min(forwardCornerCount(a, b, pathN), forwardCornerCount(b, a, pathN));
}

// Left/right along sweep line direction (`lineDir`): smaller dot(lineDir) = left end of chord.
struct ComponentLeftRightShells {
  std::vector<BoundaryAnchor> left;
  std::vector<BoundaryAnchor> right;
};

ComponentLeftRightShells buildComponentLeftRightShells(const RegionGeometry& region,
                                                       const QPointF& lineDir,
                                                       const std::vector<IslandGraphNode>& nodes,
                                                       const std::vector<int>& compNodes) {
  ComponentLeftRightShells out;
  for (int nid : compNodes) {
    if (nid < 0 || nid >= static_cast<int>(nodes.size())) continue;
    const StripeSegment& seg = nodes[static_cast<size_t>(nid)].seg;
    const double ta = QPointF::dotProduct(seg.a, lineDir);
    const double tb = QPointF::dotProduct(seg.b, lineDir);
    const QPointF& leftPt = (ta <= tb) ? seg.a : seg.b;
    const QPointF& rightPt = (ta <= tb) ? seg.b : seg.a;
    insertBoundaryAnchorForPoint(region, leftPt, out.left);
    insertBoundaryAnchorForPoint(region, rightPt, out.right);
  }
  return out;
}

bool shellAnchorsCloseByContour(const RegionGeometry& region, const std::vector<BoundaryAnchor>& a,
                                const std::vector<BoundaryAnchor>& b) {
  constexpr int kMaxMergeBoundaryCorners = 1;
  if (a.empty() || b.empty()) return false;
  for (const BoundaryAnchor& aa : a) {
    for (const BoundaryAnchor& bb : b) {
      if (shortestCornerCount(region, aa, bb) <= kMaxMergeBoundaryCorners)
        return true;
    }
  }
  return false;
}

// Hard floors for the centroid proximity gate: protect merge from collapsing
// on very tight steps where step*factor would be < ~1m.
constexpr double kComponentMergeAlongMinMeters = 6.0;
constexpr double kComponentMergeAcrossMinMeters = 10.0;
// "Bypass merge" hard cap: a tiny stranded fragment is absorbed into a
// neighbor only if it is closer than this (covers very large steps too).
constexpr double kBypassMergeMinMeters = 20.0;
constexpr double kBypassMergeStepFactor = 8.0;

bool componentTouchesExclusion(const RegionGeometry& insetGeo,
                               const RegionGeometry& workingGeo,
                               const std::vector<IslandGraphNode>& nodes,
                               const std::vector<int>& compNodes, double stepMeters) {
  for (int nid : compNodes) {
    if (nid < 0 || nid >= static_cast<int>(nodes.size())) continue;
    if (stripeTouchesExclusionBoundary(insetGeo, workingGeo,
                                       nodes[static_cast<size_t>(nid)].seg, stepMeters)) {
      return true;
    }
  }
  return false;
}

std::pair<double, double> componentAlongSpan(const std::vector<int>& compNodes,
                                             const std::vector<IslandGraphNode>& nodes,
                                             const QPointF& lineDir) {
  double lo = std::numeric_limits<double>::max();
  double hi = -std::numeric_limits<double>::max();
  for (int nid : compNodes) {
    if (nid < 0 || nid >= static_cast<int>(nodes.size())) continue;
    const StripeSegment& s = nodes[static_cast<size_t>(nid)].seg;
    const double ta = QPointF::dotProduct(s.a, lineDir);
    const double tb = QPointF::dotProduct(s.b, lineDir);
    lo = std::min(lo, std::min(ta, tb));
    hi = std::max(hi, std::max(ta, tb));
  }
  return {lo, hi};
}

bool alongIntervalsOverlap(double aLo, double aHi, double bLo, double bHi, double eps = 0.5) {
  return std::min(aHi, bHi) > std::max(aLo, bLo) + eps;
}

// Reject merge when another component occupies the row gap between A and B
// in the shared along-line corridor (e.g. a red strip between purple slabs).
bool interveningComponentBlocksMerge(int compA, int compB, int minRowA, int maxRowA,
                                     int minRowB, int maxRowB,
                                     const std::vector<std::pair<int, int>>& compRowSpans,
                                     const std::vector<std::vector<int>>& components,
                                     const std::vector<IslandGraphNode>& nodes,
                                     const QPointF& lineDir) {
  int gapMinRow = 0;
  int gapMaxRow = -1;
  if (maxRowA < minRowB) {
    gapMinRow = maxRowA + 1;
    gapMaxRow = minRowB - 1;
  } else if (maxRowB < minRowA) {
    gapMinRow = maxRowB + 1;
    gapMaxRow = minRowA - 1;
  } else {
    return false;
  }
  if (gapMinRow > gapMaxRow) return false;

  const auto spanA = componentAlongSpan(components[static_cast<size_t>(compA)], nodes, lineDir);
  const auto spanB = componentAlongSpan(components[static_cast<size_t>(compB)], nodes, lineDir);
  const double bandLo = std::max(spanA.first, spanB.first);
  const double bandHi = std::min(spanA.second, spanB.second);
  if (bandHi <= bandLo + 1e-3) return false;

  for (int ci = 0; ci < static_cast<int>(components.size()); ++ci) {
    if (ci == compA || ci == compB) continue;
    const auto [cMinRow, cMaxRow] = compRowSpans[static_cast<size_t>(ci)];
    if (cMaxRow < gapMinRow || cMinRow > gapMaxRow) continue;
    const auto spanC = componentAlongSpan(components[static_cast<size_t>(ci)], nodes, lineDir);
    if (alongIntervalsOverlap(spanC.first, spanC.second, bandLo, bandHi)) return true;
  }
  return false;
}
}  // namespace

bool runStageMerge(const RoutePipelineInput& input, RoutePipelineState& state,
                   RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Merge);

  std::vector<std::vector<int>> components = state.rawComponents;
  if (components.empty()) {
    state.components.clear();
    state.nodeComponent.clear();
    debug.fail(QStringLiteral("Нет компонент для слияния"));
    return false;
  }

  struct CompStats {
    int minRow = std::numeric_limits<int>::max();
    int maxRow = std::numeric_limits<int>::min();
    QPointF centroid{0.0, 0.0};
    int count = 0;
    int mainPath = -1;
  };
  std::vector<CompStats> stats(components.size());
  for (int ci = 0; ci < static_cast<int>(components.size()); ++ci) {
    auto& st = stats[static_cast<size_t>(ci)];
    std::map<int, int> pathFreq;
    for (int nodeId : components[static_cast<size_t>(ci)]) {
      const auto& s = state.nodes[static_cast<size_t>(nodeId)].seg;
      st.minRow = std::min(st.minRow, s.row);
      st.maxRow = std::max(st.maxRow, s.row);
      st.centroid += s.mid;
      ++st.count;
      if (s.pathIndex >= 0) pathFreq[s.pathIndex]++;
    }
    if (st.count > 0) st.centroid /= static_cast<double>(st.count);
    int bestFreq = -1;
    for (const auto& [p, f] : pathFreq) {
      if (f > bestFreq) {
        bestFreq = f;
        st.mainPath = p;
      }
    }
  }

  std::vector<ComponentLeftRightShells> lrShells(components.size());
  std::vector<bool> compExclusionTouch(components.size(), false);
  for (int ci = 0; ci < static_cast<int>(components.size()); ++ci) {
    lrShells[static_cast<size_t>(ci)] = buildComponentLeftRightShells(
        state.insetGeo, state.lineDir, state.nodes, components[static_cast<size_t>(ci)]);
    compExclusionTouch[static_cast<size_t>(ci)] = componentTouchesExclusion(
        state.insetGeo, state.workingGeo, state.nodes, components[static_cast<size_t>(ci)],
        input.stepMeters);
  }

  std::vector<int> compParent(components.size());
  for (int i = 0; i < static_cast<int>(components.size()); ++i) compParent[i] = i;
  auto compFind = [&](int x) {
    int r = x;
    while (compParent[static_cast<size_t>(r)] != r) {
      r = compParent[static_cast<size_t>(r)];
    }
    while (compParent[static_cast<size_t>(x)] != x) {
      const int p = compParent[static_cast<size_t>(x)];
      compParent[static_cast<size_t>(x)] = r;
      x = p;
    }
    return r;
  };
  auto compUnion = [&](int a, int b) {
    const int ra = compFind(a);
    const int rb = compFind(b);
    if (ra != rb) compParent[static_cast<size_t>(rb)] = ra;
  };

  const int rowGapLimit = input.config.mergeRowGapLimit;
  const double centroidAlongLimit =
      std::max(kComponentMergeAlongMinMeters,
               input.stepMeters * input.config.mergeCentroidAlongFactor);
  const double centroidAcrossLimit =
      std::max(kComponentMergeAcrossMinMeters,
               input.stepMeters * input.config.mergeCentroidAcrossFactor);
  const int tinySizeLimit = input.config.mergeTinySizeLimit;
  const double bypassCentroidDistLimit =
      std::max(kBypassMergeMinMeters, input.stepMeters * kBypassMergeStepFactor);

  int regularMerges = 0;
  int bypassMerges = 0;
  int shellRejected = 0;
  int exclusionRejected = 0;
  int interveningRejected = 0;
  std::vector<std::pair<int, int>> compRowSpans(components.size());
  for (int ci = 0; ci < static_cast<int>(components.size()); ++ci) {
    compRowSpans[static_cast<size_t>(ci)] = {stats[static_cast<size_t>(ci)].minRow,
                                             stats[static_cast<size_t>(ci)].maxRow};
  }
  for (int a = 0; a < static_cast<int>(components.size()); ++a) {
    for (int b = a + 1; b < static_cast<int>(components.size()); ++b) {
      if (compExclusionTouch[static_cast<size_t>(a)] !=
          compExclusionTouch[static_cast<size_t>(b)]) {
        ++exclusionRejected;
        continue;
      }
      const auto& sa = stats[static_cast<size_t>(a)];
      const auto& sb = stats[static_cast<size_t>(b)];
      const int rowGap =
          std::max(0, std::max(sa.minRow, sb.minRow) - std::min(sa.maxRow, sb.maxRow) - 1);
      const QPointF d = sb.centroid - sa.centroid;
      const double centroidDist = std::hypot(d.x(), d.y());
      const double along = std::abs(QPointF::dotProduct(d, state.lineDir));
      const double across = std::abs(QPointF::dotProduct(d, state.normal));
      const bool closeEnough = (along <= centroidAlongLimit) && (across <= centroidAcrossLimit);
      const bool oneTiny = sa.count <= tinySizeLimit || sb.count <= tinySizeLimit;
      const bool samePath =
          (sa.mainPath >= 0 && sb.mainPath >= 0 && sa.mainPath == sb.mainPath);
      const bool localBypassMerge = oneTiny && samePath &&
                                    centroidDist <= bypassCentroidDistLimit &&
                                    directConnectorInsideRegion(state.insetGeo, sa.centroid,
                                                                sb.centroid);
      if (rowGap > rowGapLimit && !localBypassMerge) continue;
      if (!directConnectorInsideRegion(state.insetGeo, sa.centroid, sb.centroid)) {
        ++exclusionRejected;
        continue;
      }
      if (interveningComponentBlocksMerge(a, b, sa.minRow, sa.maxRow, sb.minRow, sb.maxRow,
                                          compRowSpans, components, state.nodes, state.lineDir)) {
        ++interveningRejected;
        continue;
      }
      const auto& la = lrShells[static_cast<size_t>(a)].left;
      const auto& ra = lrShells[static_cast<size_t>(a)].right;
      const auto& lb = lrShells[static_cast<size_t>(b)].left;
      const auto& rb = lrShells[static_cast<size_t>(b)].right;
      // Same corridor (fragments stacked in row): both outer sides agree.
      const bool shellStacked = shellAnchorsCloseByContour(state.insetGeo, la, lb) &&
                                shellAnchorsCloseByContour(state.insetGeo, ra, rb);
      // Neighbor slabs across sweep: shared seam is right(A)–left(B) or the mirror.
      const double acrossA = QPointF::dotProduct(sa.centroid, state.normal);
      const double acrossB = QPointF::dotProduct(sb.centroid, state.normal);
      const double acrossSepThreshold = std::max(1.0, input.stepMeters * 0.2);
      bool shellFacing = false;
      if (std::abs(acrossA - acrossB) > acrossSepThreshold) {
        if (acrossA < acrossB)
          shellFacing = shellAnchorsCloseByContour(state.insetGeo, ra, lb);
        else
          shellFacing = shellAnchorsCloseByContour(state.insetGeo, la, rb);
      } else {
        shellFacing = shellAnchorsCloseByContour(state.insetGeo, ra, lb) ||
                      shellAnchorsCloseByContour(state.insetGeo, la, rb);
      }
      const bool sameContourShell = shellStacked || shellFacing;
      if (!sameContourShell) {
        if ((closeEnough && (oneTiny || samePath)) || localBypassMerge) ++shellRejected;
        continue;
      }
      if (closeEnough && (oneTiny || samePath)) {
        compUnion(a, b);
        ++regularMerges;
      } else if (localBypassMerge) {
        compUnion(a, b);
        ++bypassMerges;
        debug.log(QStringLiteral("bypass merge a=%1 b=%2 rowGap=%3 dist=%4")
                      .arg(a).arg(b).arg(rowGap).arg(centroidDist, 0, 'f', 2));
      }
    }
  }

  std::map<int, std::vector<int>> grouped;
  for (int i = 0; i < static_cast<int>(components.size()); ++i) {
    grouped[compFind(i)].insert(grouped[compFind(i)].end(),
                                components[static_cast<size_t>(i)].begin(),
                                components[static_cast<size_t>(i)].end());
  }
  std::vector<std::vector<int>> mergedComponents;
  mergedComponents.reserve(grouped.size());
  for (auto& [_, nodesInGroup] : grouped) {
    mergedComponents.push_back(std::move(nodesInGroup));
  }
  state.components = std::move(mergedComponents);
  state.nodeComponent.assign(state.nodes.size(), -1);
  for (int ci = 0; ci < static_cast<int>(state.components.size()); ++ci) {
    for (int nodeId : state.components[static_cast<size_t>(ci)]) {
      if (nodeId >= 0 && nodeId < static_cast<int>(state.nodeComponent.size())) {
        state.nodeComponent[static_cast<size_t>(nodeId)] = ci;
      }
    }
  }

  debug.metric(QStringLiteral("rawComponents"),
               static_cast<int>(state.rawComponents.size()));
  debug.metric(QStringLiteral("mergedComponents"),
               static_cast<int>(state.components.size()));
  debug.metric(QStringLiteral("regularMerges"), regularMerges);
  debug.metric(QStringLiteral("bypassMerges"), bypassMerges);
  debug.metric(QStringLiteral("mergeShellRejected"), shellRejected);
  debug.metric(QStringLiteral("mergeExclusionRejected"), exclusionRejected);
  debug.metric(QStringLiteral("mergeInterveningRejected"), interveningRejected);
  debug.metric(QStringLiteral("alongLimit, м"), centroidAlongLimit, 1);
  debug.metric(QStringLiteral("acrossLimit, м"), centroidAcrossLimit, 1);
  debug.summarize(QStringLiteral("После слияния: %1 из %2 (объединено %3)")
                      .arg(static_cast<int>(state.components.size()))
                      .arg(static_cast<int>(state.rawComponents.size()))
                      .arg(regularMerges + bypassMerges));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
