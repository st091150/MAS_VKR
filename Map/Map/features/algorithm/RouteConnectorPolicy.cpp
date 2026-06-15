#include "RouteConnectorPolicy.h"

#include "../../utils/ClipperUtils.h"
#include "../../utils/MapLog.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace RouteAlgo {
namespace {
constexpr double kGeomEps = 1e-6;
constexpr double kSamePointDistanceMeters = 0.05;
constexpr double kShortDirectConnectorMinMeters = 0.5;
constexpr double kBoundaryReachMinMeters = 1.0;
constexpr double kBoundaryReachStepFactor = 0.75;

struct HoleOuterPair {
  int holeIdx = -1;
  int outerIdx = -1;
};

std::vector<HoleOuterPair> listHoleOuterPairs(const RegionGeometry& region) {
  std::vector<HoleOuterPair> pairs;
  for (int hi = 0; hi < static_cast<int>(region.paths.size()); ++hi) {
    const ClipperLib::Path& hole = region.paths[static_cast<size_t>(hi)];
    if (hole.size() < 3 || ClipperLib::Area(hole) >= 0.0) continue;
    const ClipperLib::IntPoint probe = hole.front();
    int bestOuter = -1;
    double bestArea = std::numeric_limits<double>::max();
    for (int oi = 0; oi < static_cast<int>(region.paths.size()); ++oi) {
      const ClipperLib::Path& outer = region.paths[static_cast<size_t>(oi)];
      if (outer.size() < 3 || ClipperLib::Area(outer) <= 0.0) continue;
      if (ClipperLib::PointInPolygon(probe, outer) == 0) continue;
      const double area = ClipperLib::Area(outer);
      if (area < bestArea) {
        bestArea = area;
        bestOuter = oi;
      }
    }
    if (bestOuter >= 0) pairs.push_back({hi, bestOuter});
  }
  return pairs;
}

enum class TransferMode { SingleRing, HoleToOuter, OuterToHole };

struct TransferPlan {
  TransferMode mode = TransferMode::SingleRing;
  double score = std::numeric_limits<double>::max();
  BoundaryAnchor singleFrom;
  BoundaryAnchor singleTo;
  int ringAIdx = -1;
  int ringBIdx = -1;
  BoundaryAnchor legAFrom;
  BoundaryAnchor legATo;
  BoundaryAnchor legBFrom;
  BoundaryAnchor legBTo;
  QPointF bridgeA;
  QPointF bridgeB;
};

void appendConnectorPath(QVector<QPointF>& projRoute, const QVector<QPointF>& connector) {
  for (int i = 1; i < connector.size(); ++i) {
    projRoute.push_back(connector[i]);
  }
}

void considerBridgeTransfer(const RegionGeometry& region, double reachLimit,
                            double maxBridgeLen, const BoundaryAnchor& startAnchor,
                            const ClipperLib::Path& startPath, int startIdx,
                            const BoundaryAnchor& endAnchor, const ClipperLib::Path& endPath,
                            int endIdx, TransferMode mode, TransferPlan& best) {
  if (startAnchor.segIndex < 0 || endAnchor.segIndex < 0) return;
  if (startAnchor.distance > reachLimit || endAnchor.distance > reachLimit) return;
  const double startPeri = region.pathPerimeters[static_cast<size_t>(startIdx)];
  const double endPeri = region.pathPerimeters[static_cast<size_t>(endIdx)];
  const int startN = static_cast<int>(startPath.size());
  const int endN = static_cast<int>(endPath.size());
  if (startN < 3 || endN < 3) return;

  auto sampleOnRing = [](const ClipperLib::Path& path, int edgeIdx, bool useMid) {
    const int n = static_cast<int>(path.size());
    const QPointF a = ClipperUtils::fromClip(path[static_cast<size_t>(edgeIdx)]);
    const QPointF b = ClipperUtils::fromClip(path[static_cast<size_t>((edgeIdx + 1) % n)]);
    return useMid ? (a + b) * 0.5 : a;
  };

  const int startStep = std::max(1, startN / 48);
  const int endStep = std::max(1, endN / 48);
  for (int hi = 0; hi < startN; hi += startStep) {
    for (bool useStartMid : {false, true}) {
      const QPointF hp = sampleOnRing(startPath, hi, useStartMid);
      const BoundaryAnchor ha = nearestBoundaryAnchor(startPath, hp, startIdx);
      if (ha.segIndex < 0) continue;
      for (int oi = 0; oi < endN; oi += endStep) {
        for (bool useEndMid : {false, true}) {
          const QPointF op = sampleOnRing(endPath, oi, useEndMid);
          if (!directConnectorInsideRegion(region, hp, op)) continue;
          const QPointF bridgeMid((hp.x() + op.x()) * 0.5, (hp.y() + op.y()) * 0.5);
          if (!regionContainsPoint(region, bridgeMid)) continue;
          const BoundaryAnchor oa = nearestBoundaryAnchor(endPath, op, endIdx);
          if (oa.segIndex < 0) continue;
          const double arcA =
              shortestBoundaryArcDistance(startPath, startAnchor, ha, startPeri);
          const double arcB =
              shortestBoundaryArcDistance(endPath, oa, endAnchor, endPeri);
          if (!std::isfinite(arcA) || !std::isfinite(arcB)) continue;
          const double bridgeLen = std::hypot((op - hp).x(), (op - hp).y());
          if (bridgeLen > maxBridgeLen) continue;
          const double score =
              startAnchor.distance + arcA + bridgeLen + arcB + endAnchor.distance;
          if (score >= best.score) continue;
          best.mode = mode;
          best.score = score;
          best.ringAIdx = startIdx;
          best.ringBIdx = endIdx;
          best.legAFrom = startAnchor;
          best.legATo = ha;
          best.legBFrom = oa;
          best.legBTo = endAnchor;
          best.bridgeA = hp;
          best.bridgeB = op;
        }
      }
    }
  }
}

void chooseSingleRingTransfer(const RegionGeometry& region, const QPointF& fromPoint,
                              const QPointF& toPoint, double reachLimit, TransferPlan& best) {
  const BoundaryAnchor nearestFrom = nearestBoundaryAnchorOnRegion(region, fromPoint);
  const BoundaryAnchor nearestTo = nearestBoundaryAnchorOnRegion(region, toPoint);
  if (nearestFrom.pathIndex < 0 || nearestTo.pathIndex < 0 ||
      nearestFrom.pathIndex != nearestTo.pathIndex ||
      nearestFrom.distance > reachLimit || nearestTo.distance > reachLimit) {
    return;
  }
  const int p = nearestFrom.pathIndex;
  if (p < 0 || p >= static_cast<int>(region.paths.size()) || region.paths[p].size() < 3) return;
  const BoundaryAnchor from = nearestBoundaryAnchor(region.paths[p], fromPoint, p);
  const BoundaryAnchor to = nearestBoundaryAnchor(region.paths[p], toPoint, p);
  if (from.segIndex < 0 || to.segIndex < 0) return;
  if (from.distance > reachLimit || to.distance > reachLimit) return;
  const double arc =
      shortestBoundaryArcDistance(region.paths[p], from, to, region.pathPerimeters[p]);
  if (!std::isfinite(arc)) return;
  const double score = from.distance + to.distance + arc;
  if (score >= best.score) return;
  best.mode = TransferMode::SingleRing;
  best.score = score;
  best.singleFrom = from;
  best.singleTo = to;
}

TransferPlan chooseInterIslandTransfer(const RegionGeometry& region, const QPointF& fromPoint,
                                       const QPointF& toPoint, double stepMeters,
                                       double shortDirectMaxMeters) {
  TransferPlan best;
  best.score = std::numeric_limits<double>::max();
  const double reachLimit =
      std::max(kBoundaryReachMinMeters, stepMeters * kBoundaryReachStepFactor);
  const double maxBridgeLen =
      std::max(kShortDirectConnectorMinMeters,
               std::min(stepMeters * 2.5, shortDirectMaxMeters));
  chooseSingleRingTransfer(region, fromPoint, toPoint, reachLimit, best);

  for (const HoleOuterPair& pair : listHoleOuterPairs(region)) {
    const ClipperLib::Path& holePath = region.paths[static_cast<size_t>(pair.holeIdx)];
    const ClipperLib::Path& outerPath = region.paths[static_cast<size_t>(pair.outerIdx)];
    if (holePath.size() < 3 || outerPath.size() < 3) continue;

    const BoundaryAnchor fromHole = nearestBoundaryAnchor(holePath, fromPoint, pair.holeIdx);
    const BoundaryAnchor toOuter = nearestBoundaryAnchor(outerPath, toPoint, pair.outerIdx);
    considerBridgeTransfer(region, reachLimit, maxBridgeLen, fromHole, holePath, pair.holeIdx,
                           toOuter, outerPath, pair.outerIdx, TransferMode::HoleToOuter, best);

    const BoundaryAnchor fromOuter = nearestBoundaryAnchor(outerPath, fromPoint, pair.outerIdx);
    const BoundaryAnchor toHole = nearestBoundaryAnchor(holePath, toPoint, pair.holeIdx);
    considerBridgeTransfer(region, reachLimit, maxBridgeLen, fromOuter, outerPath, pair.outerIdx,
                           toHole, holePath, pair.holeIdx, TransferMode::OuterToHole, best);
  }
  return best;
}
}  // namespace

RouteConnectorPolicy::RouteConnectorPolicy(const RegionGeometry& region,
                                           const RouteAlgorithmConfig& config, double stepMeters,
                                           double invalidConnectorCost, bool routeDebug)
    : mRegion(region),
      mConfig(config),
      mStepMeters(stepMeters),
      mInvalidConnectorCost(invalidConnectorCost),
      mRouteDebug(routeDebug) {}

double RouteConnectorPolicy::directLengthCap(ConnectorHopKind kind) const {
  const double shortCap =
      std::max(kShortDirectConnectorMinMeters, mConfig.shortDirectConnectorMaxMeters);
  if (kind == ConnectorHopKind::IntraIslandSnake) {
    // Row turns only: a long direct hop is almost always a diagonal across a
    // concave pocket or cutout, not a legitimate boustrophedon step.
    const double rowCap = std::max(shortCap, mStepMeters * 1.35);
    const double configCap =
        std::max(kShortDirectConnectorMinMeters, mConfig.intraIslandDirectConnectorMaxMeters);
    return std::min(configCap, rowCap);
  }
  // Inter-island hops never use direct shortcuts — always boundary / bridge.
  return 0.0;
}

bool RouteConnectorPolicy::isDirectInside(const QPointF& a, const QPointF& b) const {
  return directConnectorInsideRegion(mRegion, a, b);
}

void RouteConnectorPolicy::chooseBestBoundaryTransfer(const QPointF& fromPoint, const QPointF& toPoint,
                                                      BoundaryAnchor& bestFrom, BoundaryAnchor& bestTo,
                                                      double& bestScore) const {
  bestScore = std::numeric_limits<double>::max();
  const double reachLimit =
      std::max(kBoundaryReachMinMeters, mStepMeters * kBoundaryReachStepFactor);
  const BoundaryAnchor nearestFrom = nearestBoundaryAnchorOnRegion(mRegion, fromPoint);
  const BoundaryAnchor nearestTo = nearestBoundaryAnchorOnRegion(mRegion, toPoint);
  if (nearestFrom.pathIndex < 0 || nearestTo.pathIndex < 0 ||
      nearestFrom.pathIndex != nearestTo.pathIndex ||
      nearestFrom.distance > reachLimit || nearestTo.distance > reachLimit) {
    return;
  }
  for (int p = 0; p < static_cast<int>(mRegion.paths.size()); ++p) {
    if (p != nearestFrom.pathIndex) continue;
    if (mRegion.paths[p].size() < 3) continue;
    const auto from = nearestBoundaryAnchor(mRegion.paths[p], fromPoint, p);
    const auto to = nearestBoundaryAnchor(mRegion.paths[p], toPoint, p);
    if (from.segIndex < 0 || to.segIndex < 0) continue;
    if (from.distance > reachLimit || to.distance > reachLimit) continue;
    const double arc =
        shortestBoundaryArcDistance(mRegion.paths[p], from, to, mRegion.pathPerimeters[p]);
    if (!std::isfinite(arc)) continue;
    const double score = from.distance + to.distance + arc;
    if (score < bestScore) {
      bestScore = score;
      bestFrom = from;
      bestTo = to;
    }
  }
}

double RouteConnectorPolicy::connectorCost(const QPointF& from, const QPointF& to,
                                           ConnectorHopKind kind) const {
  const double directLen = std::hypot((to - from).x(), (to - from).y());
  const bool directInside = isDirectInside(from, to);
  const double shortDirectLimit = directLengthCap(kind);
  if (directInside && directLen <= shortDirectLimit) {
    return directLen;
  }

  if (kind == ConnectorHopKind::InterIsland) {
    const TransferPlan plan = chooseInterIslandTransfer(
        mRegion, from, to, mStepMeters,
        std::max(kShortDirectConnectorMinMeters, mConfig.shortDirectConnectorMaxMeters));
    if (std::isfinite(plan.score) && plan.score < mInvalidConnectorCost) {
      return plan.score;
    }
    return mInvalidConnectorCost;
  }

  BoundaryAnchor bf;
  BoundaryAnchor bt;
  double boundaryScore = std::numeric_limits<double>::max();
  chooseBestBoundaryTransfer(from, to, bf, bt, boundaryScore);
  if (bf.segIndex >= 0 && bt.segIndex >= 0 && std::isfinite(boundaryScore)) {
    return boundaryScore;
  }
  return mInvalidConnectorCost;
}

bool RouteConnectorPolicy::appendConnectorProj(QVector<QPointF>& projRoute, const QPointF& from,
                                               const QPointF& to, ConnectorHopKind kind) const {
  const double directLen = std::hypot((to - from).x(), (to - from).y());
  if (directLen < kSamePointDistanceMeters) return true;
  const double shortDirectLimit = directLengthCap(kind);
  const bool directInside = isDirectInside(from, to);

  if (directInside && directLen <= shortDirectLimit) {
    projRoute.push_back(to);
    return true;
  }

  if (kind == ConnectorHopKind::InterIsland) {
    const TransferPlan plan = chooseInterIslandTransfer(
        mRegion, from, to, mStepMeters,
        std::max(kShortDirectConnectorMinMeters, mConfig.shortDirectConnectorMaxMeters));
    if (!std::isfinite(plan.score) || plan.score >= mInvalidConnectorCost) {
      if (mRouteDebug) {
        qCDebug(logPipeline) << "[route-connector] inter-island transfer unavailable"
                 << "directLen=" << directLen;
      }
      return false;
    }

    if (plan.mode == TransferMode::SingleRing) {
      const QVector<QPointF> connector = buildBoundaryConnector(
          mRegion.paths[plan.singleFrom.pathIndex], plan.singleFrom, plan.singleTo,
          mRegion.pathPerimeters[static_cast<size_t>(plan.singleFrom.pathIndex)]);
      appendConnectorPath(projRoute, connector);
      return connector.size() >= 2;
    }

    const ClipperLib::Path& pathA = mRegion.paths[static_cast<size_t>(plan.ringAIdx)];
    const ClipperLib::Path& pathB = mRegion.paths[static_cast<size_t>(plan.ringBIdx)];
    const QVector<QPointF> legA = buildBoundaryConnector(
        pathA, plan.legAFrom, plan.legATo,
        mRegion.pathPerimeters[static_cast<size_t>(plan.ringAIdx)]);
    appendConnectorPath(projRoute, legA);
    if (std::hypot((plan.bridgeA - projRoute.back()).x(), (plan.bridgeA - projRoute.back()).y()) >
        kSamePointDistanceMeters) {
      projRoute.push_back(plan.bridgeA);
    }
    if (std::hypot((plan.bridgeB - projRoute.back()).x(), (plan.bridgeB - projRoute.back()).y()) >
        kSamePointDistanceMeters) {
      projRoute.push_back(plan.bridgeB);
    }
    const QVector<QPointF> legB = buildBoundaryConnector(
        pathB, plan.legBFrom, plan.legBTo,
        mRegion.pathPerimeters[static_cast<size_t>(plan.ringBIdx)]);
    appendConnectorPath(projRoute, legB);
    if (std::hypot((to - projRoute.back()).x(), (to - projRoute.back()).y()) >
        kSamePointDistanceMeters) {
      projRoute.push_back(to);
    }
    if (mRouteDebug) {
      qCDebug(logPipeline) << "[route-connector] inter-island hole/outer bridge"
               << "mode=" << static_cast<int>(plan.mode)
               << "score=" << plan.score;
    }
    return projRoute.size() >= 2;
  }

  BoundaryAnchor bestFrom;
  BoundaryAnchor bestTo;
  double bestScore = std::numeric_limits<double>::max();
  chooseBestBoundaryTransfer(from, to, bestFrom, bestTo, bestScore);
  const bool haveBoundaryPath = bestFrom.segIndex >= 0 && bestTo.segIndex >= 0;

  if (mRouteDebug) {
    qCDebug(logPipeline) << "[route-connector]"
             << "directInside=" << directInside
             << "directLen=" << directLen
             << "boundaryScore=" << bestScore
             << "haveBoundary=" << haveBoundaryPath;
  }

  if (haveBoundaryPath) {
    const QVector<QPointF> connector = buildBoundaryConnector(
        mRegion.paths[bestFrom.pathIndex], bestFrom, bestTo,
        mRegion.pathPerimeters[static_cast<size_t>(bestFrom.pathIndex)]);
    appendConnectorPath(projRoute, connector);
    return connector.size() >= 2;
  }
  if (mRouteDebug) {
    qCDebug(logPipeline) << "[route-connector] no boundary path available"
             << "directLen=" << directLen;
  }
  return false;
}

}  // namespace RouteAlgo
