#include "RoutePipelineVisualizer.h"

#include "../../../geometry/GeoPolyline.h"
#include "../../../geometry/GeoSegmentBatch.h"
#include "../../../utils/ClipperUtils.h"

#include <QGeoView/QGVLayer.h>
#include <QGeoView/QGVMap.h>
#include <rectangle.h>

#include <QtMath>

#include <algorithm>
#include <cmath>
#include <map>
#include <queue>
#include <set>

namespace RouteAlgo {

namespace {
// Debug-only: skip expensive adjacency overlay on large graphs.
constexpr int kMaxFaintGraphEdges = 180;

QColor componentColor(int idx) {
  const int hue = (idx * 57) % 360;
  return QColor::fromHsv(hue, 255, 255, 230);
}

QColor groupColor(int idx) {
  // Slightly different palette to distinguish "DSU group" coloring from
  // raw component coloring.
  const int hue = (idx * 67 + 23) % 360;
  return QColor::fromHsv(hue, 215, 240, 230);
}
}  // namespace

RoutePipelineVisualizer::RoutePipelineVisualizer(QGVMap* map, QGVLayer* layer,
                                                 const RoutePipelineState& state,
                                                 double stepMeters)
    : mMap(map), mLayer(layer), mState(&state), mStepMeters(std::max(0.1, stepMeters)) {}

void RoutePipelineVisualizer::clear() {
  if (mLayer) mLayer->deleteItems();
}

void RoutePipelineVisualizer::drawSegment(const QPointF& a, const QPointF& b,
                                          const QColor& color, double width) {
  if (!mMap || !mLayer) return;
  auto* poly = new GeoPolyline(mMap);
  QPen pen(color);
  pen.setWidthF(width);
  poly->setPen(pen);
  QVector<QGV::GeoPos> geo;
  geo.push_back(mMap->getProjection()->projToGeo(a));
  geo.push_back(mMap->getProjection()->projToGeo(b));
  poly->points = geo;
  mLayer->addItem(poly);
}

void RoutePipelineVisualizer::drawSegments(
    const std::vector<std::pair<QPointF, QPointF>>& segments,
    const QColor& color, double width) {
  if (!mMap || !mLayer || segments.empty()) return;
  auto* batch = new GeoSegmentBatch(mMap);
  QPen pen(color);
  pen.setWidthF(width);
  batch->setPen(pen);
  batch->segments.reserve(static_cast<int>(segments.size()));
  auto* projection = mMap->getProjection();
  if (!projection) {
    delete batch;
    return;
  }
  for (const auto& segment : segments) {
    batch->segments.push_back({projection->projToGeo(segment.first),
                               projection->projToGeo(segment.second)});
  }
  mLayer->addItem(batch);
}

void RoutePipelineVisualizer::drawPolyline(const QVector<QPointF>& pts, const QPen& pen) {
  if (!mMap || !mLayer || pts.size() < 2) return;
  auto* poly = new GeoPolyline(mMap);
  poly->setPen(pen);
  QVector<QGV::GeoPos> geo;
  geo.reserve(pts.size());
  for (const auto& p : pts) geo.push_back(mMap->getProjection()->projToGeo(p));
  poly->points = geo;
  mLayer->addItem(poly);
}

void RoutePipelineVisualizer::drawCross(const QPointF& c, const QColor& color,
                                        double size, double width) {
  drawSegment(c + QPointF(-size, 0.0), c + QPointF(size, 0.0), color, width);
  drawSegment(c + QPointF(0.0, -size), c + QPointF(0.0, size), color, width);
}

void RoutePipelineVisualizer::drawMarker(const QPointF& c, const QColor& color,
                                         double sizeMeters) {
  if (!mMap || !mLayer) return;
  const QGV::GeoPos center = mMap->getProjection()->projToGeo(c);
  const double dLat = sizeMeters / 111000.0;
  const double dLon =
      sizeMeters / (111000.0 * std::cos(qDegreesToRadians(center.latitude())));
  const QGV::GeoPos p1(center.latitude() - dLat / 2, center.longitude() - dLon / 2);
  const QGV::GeoPos p2(center.latitude() + dLat / 2, center.longitude() + dLon / 2);
  auto* rect = new Rectangle(QGV::GeoRect(p1, p2), color);
  rect->setZValue(1200);
  mLayer->addItem(rect);
}

void RoutePipelineVisualizer::drawTransition(const QPointF& a, const QPointF& b,
                                             const QColor& color, double width) {
  const double len = std::hypot((b - a).x(), (b - a).y());
  const double maxStraight = std::max(8.0, mStepMeters * 5.0);
  if (len <= maxStraight) {
    drawSegment(a, b, color, width);
    return;
  }
  // Mark endpoints when the transition would be misleadingly long.
  const double mark = std::max(1.0, mStepMeters * 0.6);
  drawCross(a, color, mark, width);
  drawCross(b, color, mark, width);
}

void RoutePipelineVisualizer::drawContextOverlay() {
  if (!mState || mState->insetRegion.empty()) return;
  int outerCount = 0;
  for (const auto& path : mState->insetRegion) {
    if (path.size() >= 3 && ClipperLib::Area(path) > 0.0) ++outerCount;
  }
  if (outerCount > 1) {
    int outerIdx = 0;
    for (const auto& path : mState->insetRegion) {
      if (path.size() < 3) continue;
      QVector<QPointF> pts = ClipperUtils::fromClipPath(path);
      const bool isOuter = ClipperLib::Area(path) > 0.0;
      const QColor color =
          isOuter ? QColor::fromHsv((outerIdx++ * 57) % 360, 255, 255, 255)
                  : QColor(210, 220, 245, 210);
      QPen pen(color);
      pen.setWidthF(isOuter ? 4.0 : 1.2);
      pen.setStyle(Qt::SolidLine);
      drawPolyline(pts, pen);
    }
  } else {
    QPen contextPen(QColor(150, 200, 255, 110));
    contextPen.setWidthF(0.7);
    addAreaOutlines(mMap, mLayer, mState->insetRegion, contextPen);
  }
  drawMarker(mState->nearestPt, QColor(255, 230, 80, 230), 0.9);
}

int RoutePipelineVisualizer::stageStepCount(PipelineStage stage) const {
  if (!mState) return 1;
  auto clamp1 = [](int v) { return std::max(1, v); };
  switch (stage) {
    case PipelineStage::Prepare:  return 1;
    case PipelineStage::Sweep:    return clamp1(static_cast<int>(mState->rawStripes.size()));
    case PipelineStage::Filter:
      return clamp1(static_cast<int>(mState->stripes.size() + mState->droppedStripes.size()));
    case PipelineStage::Graph:    return clamp1(static_cast<int>(mState->nodes.size()));
    case PipelineStage::Merge:    return clamp1(static_cast<int>(mState->components.size()));
    case PipelineStage::Routes:   return clamp1(static_cast<int>(mState->routedChunks.size()));
    case PipelineStage::Group:    return clamp1(static_cast<int>(mState->islands.size()));
    case PipelineStage::Stitch:
      // One step per island stitched; equals islandLinks + 1 in the success path.
      return clamp1(static_cast<int>(mState->stitchResult.transitionPointOffsets.size()));
    case PipelineStage::Approach:
      return 3;
    case PipelineStage::Count:    break;
  }
  return 1;
}

void RoutePipelineVisualizer::drawStage(PipelineStage stage, int stepLimit) {
  if (!mMap || !mLayer || !mState) return;
  clear();
  drawContextOverlay();
  const int total = stageStepCount(stage);
  const int limit = (stepLimit < 0) ? total : std::clamp(stepLimit, 0, total);
  switch (stage) {
    case PipelineStage::Prepare: {
      // Inset region + anchor + start direction tick.
      drawCross(mState->nearestPt + mState->lineDir * mStepMeters,
                QColor(255, 230, 80, 230), 1.0, 1.5);
      break;
    }
    case PipelineStage::Sweep: {
      const QColor kept(150, 150, 150, 170);
      const int n = std::min(limit, static_cast<int>(mState->rawStripes.size()));
      std::vector<std::pair<QPointF, QPointF>> segments;
      segments.reserve(static_cast<size_t>(n));
      for (int i = 0; i < n; ++i) {
        const auto& s = mState->rawStripes[static_cast<size_t>(i)];
        segments.push_back({s.a, s.b});
      }
      drawSegments(segments, kept, 0.9);
      break;
    }
    case PipelineStage::Filter: {
      // Step semantics: first kept stripes grow one by one; once kept are
      // exhausted, dropped stripes start appearing in red.
      const QColor keep(120, 200, 120, 180);
      const QColor drop(220, 70, 70, 200);
      const int keptN = std::min(limit, static_cast<int>(mState->stripes.size()));
      std::vector<std::pair<QPointF, QPointF>> keptSegments;
      keptSegments.reserve(static_cast<size_t>(keptN));
      for (int i = 0; i < keptN; ++i) {
        const auto& s = mState->stripes[static_cast<size_t>(i)];
        keptSegments.push_back({s.a, s.b});
      }
      const int dropN = std::clamp(limit - static_cast<int>(mState->stripes.size()),
                                   0, static_cast<int>(mState->droppedStripes.size()));
      std::vector<std::pair<QPointF, QPointF>> droppedSegments;
      droppedSegments.reserve(static_cast<size_t>(dropN));
      for (int i = 0; i < dropN; ++i) {
        const auto& s = mState->droppedStripes[static_cast<size_t>(i)];
        droppedSegments.push_back({s.a, s.b});
      }
      drawSegments(keptSegments, keep, 1.0);
      drawSegments(droppedSegments, drop, 1.0);
      break;
    }
    case PipelineStage::Graph: {
      std::map<int, int> nodeToRawComp;
      for (int ci = 0; ci < static_cast<int>(mState->rawComponents.size()); ++ci) {
        for (int n : mState->rawComponents[static_cast<size_t>(ci)]) {
          nodeToRawComp[n] = ci;
        }
      }
      const int n = std::min(limit, static_cast<int>(mState->nodes.size()));
      std::map<int, std::vector<std::pair<QPointF, QPointF>>> nodeSegmentsByComp;
      for (int i = 0; i < n; ++i) {
        const auto& node = mState->nodes[static_cast<size_t>(i)];
        const int compId = nodeToRawComp.count(i) ? nodeToRawComp[i] : 0;
        nodeSegmentsByComp[compId].push_back({node.seg.a, node.seg.b});
      }
      for (const auto& [compId, segments] : nodeSegmentsByComp) {
        drawSegments(segments, componentColor(compId), 1.4);
      }
      // Full adjacency is a *multigraph* (up to top-K edges per node) — it is
      // not the snake order. Draw candidate edges faintly, and overlay a BFS
      // spanning tree so the debug view reads like a single backbone per
      // component instead of "one-to-many stars".
      if (n <= kMaxFaintGraphEdges) {
        const QColor faintEdge(200, 200, 200, 55);
        std::vector<std::pair<QPointF, QPointF>> faintEdges;
        for (int i = 0; i < n; ++i) {
          const auto& node = mState->nodes[static_cast<size_t>(i)];
          for (int adj : node.adjStrict) {
            if (adj <= i || adj >= n) continue;
            const auto& other = mState->nodes[static_cast<size_t>(adj)].seg;
            faintEdges.push_back({node.seg.mid, other.mid});
          }
        }
        drawSegments(faintEdges, faintEdge, 0.45);
      }
      auto alongMin = [&](const StripeSegment& s) {
        const double ta = QPointF::dotProduct(s.a, mState->lineDir);
        const double tb = QPointF::dotProduct(s.b, mState->lineDir);
        return std::min(ta, tb);
      };
      for (const auto& comp : mState->rawComponents) {
        if (comp.empty()) continue;
        int root = comp.front();
        for (size_t k = 1; k < comp.size(); ++k) {
          const int nid = comp[k];
          const auto& sr = mState->nodes[static_cast<size_t>(root)].seg;
          const auto& sn = mState->nodes[static_cast<size_t>(nid)].seg;
          const double mr = alongMin(sr);
          const double mn = alongMin(sn);
          if (sn.row < sr.row || (sn.row == sr.row && mn < mr)) {
            root = nid;
          }
        }
        std::vector<int> parent(mState->nodes.size(), -1);
        std::queue<int> q;
        if (root >= 0 && root < static_cast<int>(mState->nodes.size())) {
          q.push(root);
          parent[static_cast<size_t>(root)] = root;
        }
        while (!q.empty()) {
          const int u = q.front();
          q.pop();
          if (u < 0 || u >= static_cast<int>(mState->nodes.size())) continue;
          for (int v : mState->nodes[static_cast<size_t>(u)].adjStrict) {
            if (v < 0 || v >= static_cast<int>(mState->nodes.size())) continue;
            if (parent[static_cast<size_t>(v)] >= 0) continue;
            parent[static_cast<size_t>(v)] = u;
            q.push(v);
          }
        }
        const QColor treeColor(255, 255, 255, 200);
        std::vector<std::pair<QPointF, QPointF>> treeEdges;
        for (int nid : comp) {
          if (nid < 0 || nid >= n) continue;
          const int p = parent[static_cast<size_t>(nid)];
          if (p < 0 || p == nid) continue;
          if (p >= n) continue;
          const auto& a = mState->nodes[static_cast<size_t>(nid)].seg.mid;
          const auto& b = mState->nodes[static_cast<size_t>(p)].seg.mid;
          treeEdges.push_back({a, b});
        }
        drawSegments(treeEdges, treeColor, 1.05);
      }
      break;
    }
    case PipelineStage::Merge: {
      const int n = std::min(limit, static_cast<int>(mState->components.size()));
      for (int ci = 0; ci < n; ++ci) {
        const QColor c = componentColor(ci);
        std::vector<std::pair<QPointF, QPointF>> segments;
        segments.reserve(mState->components[static_cast<size_t>(ci)].size());
        for (int idx : mState->components[static_cast<size_t>(ci)]) {
          const auto& node = mState->nodes[static_cast<size_t>(idx)];
          segments.push_back({node.seg.a, node.seg.b});
        }
        drawSegments(segments, c, 1.6);
      }
      break;
    }
    case PipelineStage::Routes: {
      const int n = std::min(limit, static_cast<int>(mState->routedChunks.size()));
      for (int i = 0; i < n; ++i) {
        const auto& chunk = mState->routedChunks[static_cast<size_t>(i)];
        if (chunk.phase != 0 || chunk.pts.size() < 2) continue;
        QPen pen(componentColor(chunk.compId));
        pen.setWidthF(2.0);
        drawPolyline(chunk.pts, pen);
      }
      break;
    }
    case PipelineStage::Group: {
      const int n = std::min(limit, static_cast<int>(mState->islands.size()));
      for (int i = 0; i < n; ++i) {
        const auto& island = mState->islands[static_cast<size_t>(i)];
        if (island.phase != 0) continue;
        const QColor c = groupColor(island.compId);
        QPen pen(c);
        pen.setWidthF(2.0);
        drawPolyline(island.pts, pen);
        drawCross(island.centroid, QColor(0, 0, 0, 220), std::max(1.0, mStepMeters * 0.6), 2.0);
      }
      break;
    }
    case PipelineStage::Stitch: {
      const auto& offs = mState->stitchResult.transitionPointOffsets;
      auto isUnreachable = [&](int islandIndex) {
        return std::find(mState->stitchResult.unreachableIslandIndices.begin(),
                         mState->stitchResult.unreachableIslandIndices.end(),
                         islandIndex) != mState->stitchResult.unreachableIslandIndices.end();
      };
      for (int islandIdx = 0; islandIdx < static_cast<int>(mState->islands.size()); ++islandIdx) {
        const auto& island = mState->islands[static_cast<size_t>(islandIdx)];
        if (island.phase != 0) continue;
        const bool unreachable = isUnreachable(islandIdx);
        QPen pen(unreachable ? QColor(255, 70, 70, 240) : groupColor(island.compId));
        pen.setWidthF(unreachable ? 2.4 : 1.0);
        pen.setStyle(unreachable ? Qt::DashLine : Qt::DotLine);
        drawPolyline(island.pts, pen);
        if (unreachable) {
          drawCross(island.centroid, QColor(255, 40, 40, 240),
                    std::max(1.5, mStepMeters * 0.9), 2.4);
        }
      }
      if (limit > 0 && !offs.empty()) {
        const int islandsShown = std::min(limit, static_cast<int>(offs.size()));
        const int upTo = std::clamp(offs[static_cast<size_t>(islandsShown - 1)],
                                    1, static_cast<int>(mState->stitchedProj.size()));
        QPen routePen(QColor(50, 170, 255, 220));
        routePen.setWidthF(2.0);
        drawPolyline(mState->stitchedProj.mid(0, upTo), routePen);
      }
      break;
    }
    case PipelineStage::Approach: {
      QPen routePen(QColor(50, 170, 255, 220));
      routePen.setWidthF(2.0);
      if (limit >= total && mState->fullRouteProj.size() >= 2) {
        drawPolyline(mState->fullRouteProj, routePen);
      } else {
        drawPolyline(mState->stitchedProj, routePen);
        if (limit >= 2 && mState->approachProj.size() > 1) {
          QPen approachPen(QColor(255, 190, 60, 230));
          approachPen.setWidthF(2.4);
          drawPolyline(mState->approachProj, approachPen);
        }
      }
      drawMarker(mState->fullRouteProj.isEmpty() ? mState->nearestPt
                                                 : mState->fullRouteProj.front(),
                 QColor(255, 215, 0, 230), 0.9);
      drawMarker(mState->fullRouteProj.isEmpty() ? mState->nearestPt
                                                 : mState->fullRouteProj.back(),
                 QColor(255, 120, 80, 230), 0.9);
      break;
    }
    case PipelineStage::Count:
      break;
  }
}

void RoutePipelineVisualizer::drawOperational(const QPen& routePen) {
  if (!mMap || !mLayer || !mState) return;
  clear();
  if (mState->fullRouteProj.size() >= 2) {
    drawPolyline(mState->fullRouteProj, routePen);
    drawMarker(mState->fullRouteProj.front(), QColor(255, 215, 0, 230), 0.9);
    drawMarker(mState->fullRouteProj.back(), QColor(255, 120, 80, 230), 0.9);
  }
}

}  // namespace RouteAlgo
