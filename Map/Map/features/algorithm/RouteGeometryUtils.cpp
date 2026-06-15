#include "RouteGeometryUtils.h"

#include "../../geometry/GeoPolyline.h"
#include "../../utils/ClipperUtils.h"

#include <QGeoView/QGVLayer.h>
#include <QColor>
#include <QPen>
#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/perimeter.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/segment.hpp>

namespace RouteAlgo {
namespace {
constexpr double kGeomEps = 1e-6;
constexpr double kStripeMinLengthMeters = 0.05;
constexpr double kBoundsFallbackMeters = 1.0;
// Tiny padding around the bbox so the clipped sweep segment crosses the
// polygon boundary cleanly (avoids degenerate corner-only intersections).
constexpr double kBBoxClipPaddingMeters = 1.0;

using BgLinestring = bg::model::linestring<BgPoint>;
using BgSegment = bg::model::segment<BgPoint>;
using BgRing = BgPolygon::ring_type;

// Clip an infinite line `p(t) = anchor + dir * t` to the (padded) axis-aligned
// bbox using the Liang-Barsky algorithm. Returns false if the line misses the
// bbox entirely. On success, `outA` / `outB` are the two segment endpoints
// strictly within the (padded) bbox.
bool clipInfiniteLineToBounds(const QPointF& anchor, const QPointF& dir,
                              const RegionBounds& bounds, double padding,
                              QPointF& outA, QPointF& outB) {
  const double minX = bounds.minX - padding;
  const double maxX = bounds.maxX + padding;
  const double minY = bounds.minY - padding;
  const double maxY = bounds.maxY + padding;

  double tMin = -std::numeric_limits<double>::infinity();
  double tMax = std::numeric_limits<double>::infinity();

  auto clipAxis = [&](double dirComp, double anchorComp, double lo, double hi) -> bool {
    if (std::abs(dirComp) < kGeomEps) {
      // Line is parallel to this axis pair: fail iff anchor is outside the slab.
      return anchorComp >= lo && anchorComp <= hi;
    }
    double t1 = (lo - anchorComp) / dirComp;
    double t2 = (hi - anchorComp) / dirComp;
    if (t1 > t2) std::swap(t1, t2);
    tMin = std::max(tMin, t1);
    tMax = std::min(tMax, t2);
    return tMin <= tMax;
  };

  if (!clipAxis(dir.x(), anchor.x(), minX, maxX)) return false;
  if (!clipAxis(dir.y(), anchor.y(), minY, maxY)) return false;
  if (tMin > tMax) return false;

  outA = anchor + dir * tMin;
  outB = anchor + dir * tMax;
  return true;
}

// Append Clipper path vertices as a closed boost ring.
void fillRingFromClipperPath(const ClipperLib::Path& src, BgRing& dst) {
  dst.clear();
  dst.reserve(src.size() + 1);
  for (const auto& cp : src) {
    const QPointF q = ClipperUtils::fromClip(cp);
    dst.emplace_back(q.x(), q.y());
  }
  if (!dst.empty() &&
      (dst.front().x() != dst.back().x() || dst.front().y() != dst.back().y())) {
    dst.push_back(dst.front());
  }
}

QPointF nearestPointOnPath(const ClipperLib::Path& path, const QPointF& p) {
  QPointF best = p;
  double bestDist = std::numeric_limits<double>::max();
  const int n = static_cast<int>(path.size());
  for (int i = 0; i < n; ++i) {
    const QPointF a = ClipperUtils::fromClip(path[i]);
    const QPointF b = ClipperUtils::fromClip(path[(i + 1) % n]);
    const QPointF ab = b - a;
    const double ab2 = QPointF::dotProduct(ab, ab);
    if (ab2 < kGeomEps) continue;
    const double t = std::clamp(QPointF::dotProduct(p - a, ab) / ab2, 0.0, 1.0);
    const QPointF proj = a + ab * t;
    const double d = std::hypot((proj - p).x(), (proj - p).y());
    if (d < bestDist) {
      bestDist = d;
      best = proj;
    }
  }
  return best;
}

}  // namespace

QPointF normalizeOrZero(const QPointF& v) {
  const double len = std::hypot(v.x(), v.y());
  if (len < kGeomEps) return {0.0, 0.0};
  return {v.x() / len, v.y() / len};
}

RegionGeometry buildRegionGeometry(const ClipperLib::Paths& region) {
  RegionGeometry rg;
  rg.paths = region;
  rg.bounds = computeRegionBounds(region);

  int largestOuter = -1;
  double largestArea = -1.0;
  for (int i = 0; i < static_cast<int>(region.size()); ++i) {
    if (region[i].size() < 3) continue;
    const double a = ClipperLib::Area(region[i]);
    if (a > 0.0 && a > largestArea) {
      largestArea = a;
      largestOuter = i;
    }
  }
  rg.outerPathIndex = largestOuter;

  // Build boost multi-polygon from Clipper's PolyTree so holes stay attached
  // to the correct outer ring (flat area-sign classification can drop holes).
  rg.polygon.clear();
  if (!region.empty()) {
    ClipperLib::Clipper clp;
    ClipperLib::PolyTree tree;
    clp.AddPaths(region, ClipperLib::ptSubject, true);
    clp.Execute(ClipperLib::ctUnion, tree, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
    for (auto* outerNode : tree.Childs) {
      if (!outerNode || outerNode->Contour.size() < 3 || outerNode->IsHole()) continue;
      BgPolygon poly;
      fillRingFromClipperPath(outerNode->Contour, poly.outer());
      for (auto* child : outerNode->Childs) {
        if (!child || child->Contour.size() < 3 || !child->IsHole()) continue;
        poly.inners().emplace_back();
        fillRingFromClipperPath(child->Contour, poly.inners().back());
      }
      bg::correct(poly);
      if (!poly.outer().empty()) rg.polygon.push_back(std::move(poly));
    }
  }

  // Fallback for degenerate input that did not produce a PolyTree.
  if (rg.polygon.empty()) {
    struct OuterEntry {
      int idx;
      double absArea;
    };
    std::vector<OuterEntry> outers;
    std::vector<int> holes;
    for (int i = 0; i < static_cast<int>(region.size()); ++i) {
      if (region[i].size() < 3) continue;
      const double a = ClipperLib::Area(region[i]);
      if (a > 0.0) {
        outers.push_back({i, a});
      } else if (a < 0.0) {
        holes.push_back(i);
      }
    }
    std::vector<int> holeOwner(holes.size(), -1);
    for (size_t hi = 0; hi < holes.size(); ++hi) {
      const ClipperLib::IntPoint probe = region[holes[hi]].front();
      int bestOuter = -1;
      double bestOuterArea = std::numeric_limits<double>::max();
      for (size_t oi = 0; oi < outers.size(); ++oi) {
        const int containment = ClipperLib::PointInPolygon(probe, region[outers[oi].idx]);
        if (containment != 0 && outers[oi].absArea < bestOuterArea) {
          bestOuter = static_cast<int>(oi);
          bestOuterArea = outers[oi].absArea;
        }
      }
      holeOwner[hi] = bestOuter;
    }
    for (size_t oi = 0; oi < outers.size(); ++oi) {
      BgPolygon poly;
      fillRingFromClipperPath(region[outers[oi].idx], poly.outer());
      for (size_t hi = 0; hi < holes.size(); ++hi) {
        if (holeOwner[hi] != static_cast<int>(oi)) continue;
        poly.inners().emplace_back();
        fillRingFromClipperPath(region[holes[hi]], poly.inners().back());
      }
      bg::correct(poly);
      rg.polygon.push_back(std::move(poly));
    }
  }

  // Cache per-path perimeter using boost (closed rings).
  rg.pathPerimeters.assign(region.size(), 0.0);
  for (size_t i = 0; i < region.size(); ++i) {
    if (region[i].size() < 3) continue;
    BgRing ring;
    fillRingFromClipperPath(region[i], ring);
    rg.pathPerimeters[i] = bg::perimeter(ring);
  }
  return rg;
}

bool regionContainsPoint(const RegionGeometry& region, const QPointF& p) {
  if (region.polygon.empty()) return false;
  return bg::covered_by(BgPoint(p.x(), p.y()), region.polygon);
}

QPointF nearestPointOnRegion(const ClipperLib::Paths& region, const QPointF& p) {
  QPointF best = p;
  double bestDist = std::numeric_limits<double>::max();
  for (const auto& path : region) {
    if (path.size() < 3) continue;
    const QPointF candidate = nearestPointOnPath(path, p);
    const double d = std::hypot((candidate - p).x(), (candidate - p).y());
    if (d < bestDist) {
      bestDist = d;
      best = candidate;
    }
  }
  return best;
}

bool directConnectorInsideRegion(const RegionGeometry& region, const QPointF& a, const QPointF& b) {
  if (region.polygon.empty()) return false;
  if (!regionContainsPoint(region, a) || !regionContainsPoint(region, b)) return false;
  // Sample along the chord: concave pockets and cutout holes can fool a bare
  // two-point covered_by check on difficult floating-point geometry.
  constexpr int kSamples = 16;
  for (int i = 0; i <= kSamples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kSamples);
    const QPointF p(a.x() + (b.x() - a.x()) * t, a.y() + (b.y() - a.y()) * t);
    if (!regionContainsPoint(region, p)) return false;
  }
  return true;
}

BoundaryAnchor nearestBoundaryAnchorOnRegion(const RegionGeometry& region, const QPointF& p) {
  BoundaryAnchor best;
  best.pathIndex = -1;
  best.segIndex = -1;
  best.distance = std::numeric_limits<double>::max();
  for (int pi = 0; pi < static_cast<int>(region.paths.size()); ++pi) {
    const ClipperLib::Path& path = region.paths[static_cast<size_t>(pi)];
    if (path.size() < 3) continue;
    const BoundaryAnchor cand = nearestBoundaryAnchor(path, p, pi);
    if (cand.segIndex < 0) continue;
    if (cand.distance < best.distance) best = cand;
  }
  return best;
}

BoundaryAnchor nearestBoundaryAnchor(const ClipperLib::Path& path, const QPointF& p, int pathIndex) {
  BoundaryAnchor best;
  best.pathIndex = pathIndex;
  const int n = static_cast<int>(path.size());
  double prefix = 0.0;
  for (int i = 0; i < n; ++i) {
    const QPointF a = ClipperUtils::fromClip(path[i]);
    const QPointF b = ClipperUtils::fromClip(path[(i + 1) % n]);
    const QPointF ab = b - a;
    const double len2 = QPointF::dotProduct(ab, ab);
    const double len = std::hypot(ab.x(), ab.y());
    if (len < kGeomEps) continue;
    const double t = std::clamp(QPointF::dotProduct(p - a, ab) / len2, 0.0, 1.0);
    const QPointF proj = a + ab * t;
    const double d = std::hypot((proj - p).x(), (proj - p).y());
    if (d < best.distance) {
      best.distance = d;
      best.segIndex = i;
      best.t = t;
      best.point = proj;
      best.arc = prefix + len * t;
    }
    prefix += len;
  }
  return best;
}

QVector<QPointF> buildBoundaryConnector(const ClipperLib::Path& path, const BoundaryAnchor& from,
                                        const BoundaryAnchor& to, double cachedPerimeter) {
  QVector<QPointF> connector;
  if (from.pathIndex != to.pathIndex || from.segIndex < 0 || to.segIndex < 0) return connector;
  const int n = static_cast<int>(path.size());
  if (n < 2 || cachedPerimeter < kGeomEps) return connector;

  const double forwardArc =
      (to.arc >= from.arc) ? (to.arc - from.arc) : (to.arc + cachedPerimeter - from.arc);
  const double backwardArc = cachedPerimeter - forwardArc;
  const bool goForward = forwardArc <= backwardArc;

  connector.push_back(from.point);
  if (goForward) {
    int idx = from.segIndex;
    while (idx != to.segIndex) {
      idx = (idx + 1) % n;
      connector.push_back(ClipperUtils::fromClip(path[idx]));
    }
  } else {
    int idx = from.segIndex;
    while (idx != to.segIndex) {
      connector.push_back(ClipperUtils::fromClip(path[idx]));
      idx = (idx - 1 + n) % n;
    }
  }
  connector.push_back(to.point);
  return connector;
}

double shortestBoundaryArcDistance(const ClipperLib::Path& path, const BoundaryAnchor& from,
                                   const BoundaryAnchor& to, double cachedPerimeter) {
  if (from.pathIndex != to.pathIndex || from.segIndex < 0 || to.segIndex < 0) {
    return std::numeric_limits<double>::max();
  }
  if (cachedPerimeter < kGeomEps) return std::numeric_limits<double>::max();
  const double forwardArc =
      (to.arc >= from.arc) ? (to.arc - from.arc) : (to.arc + cachedPerimeter - from.arc);
  const double backwardArc = cachedPerimeter - forwardArc;
  Q_UNUSED(path);
  return std::min(forwardArc, backwardArc);
}

double regionSignedArea(const ClipperLib::Paths& paths) {
  double sum = 0.0;
  for (const auto& p : paths) {
    if (p.size() >= 3) sum += ClipperLib::Area(p);
  }
  return sum;
}

RegionBounds computeRegionBounds(const ClipperLib::Paths& region) {
  RegionBounds b;
  for (const auto& path : region) {
    for (const auto& ip : path) {
      const QPointF p = ClipperUtils::fromClip(ip);
      b.minX = std::min(b.minX, p.x());
      b.minY = std::min(b.minY, p.y());
      b.maxX = std::max(b.maxX, p.x());
      b.maxY = std::max(b.maxY, p.y());
    }
  }
  if (!(b.minX <= b.maxX && b.minY <= b.maxY)) {
    b.minX = b.minY = -kBoundsFallbackMeters;
    b.maxX = b.maxY = kBoundsFallbackMeters;
  }
  return b;
}

ClipperLib::Paths intersectRegionWithBand(const ClipperLib::Paths& region, const ClipperLib::Path& band) {
  if (region.empty() || band.size() < 3) return {};
  ClipperLib::Clipper c;
  c.AddPaths(region, ClipperLib::ptSubject, true);
  c.AddPath(band, ClipperLib::ptClip, true);
  ClipperLib::Paths solution;
  c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
  return ClipperUtils::unionPaths(solution);
}

std::vector<StripeSegment> intersectInfiniteLineWithRegion(const RegionGeometry& region,
                                                           const QPointF& linePoint,
                                                           const QPointF& lineDirUnit, double dValue,
                                                           int rowIndex) {
  std::vector<StripeSegment> out;
  if (region.polygon.empty()) return out;

  // 1. Anchor the sweep line `{p : p · normal = dValue}` somewhere on it.
  //    Projecting the bbox center onto the line guarantees we sit in the
  //    same coordinate range as the polygon (Web Mercator places real
  //    polygons millions of meters from the origin, so naive `normal * d`
  //    anchors fall outside any reasonable bbox).
  Q_UNUSED(linePoint);
  const QPointF normalDir(-lineDirUnit.y(), lineDirUnit.x());
  const QPointF bboxCenter((region.bounds.minX + region.bounds.maxX) * 0.5,
                           (region.bounds.minY + region.bounds.maxY) * 0.5);
  const double shift = dValue - QPointF::dotProduct(bboxCenter, normalDir);
  const QPointF lineAnchor = bboxCenter + normalDir * shift;

  // 2. Clip the line to the polygon's bbox. We always work with a finite
  //    segment that lies entirely inside the working-area bounding rectangle
  //    — easier to reason about and far more robust than feeding boost a
  //    "very long" line and relying on it to clip the result correctly.
  QPointF segA;
  QPointF segB;
  if (!clipInfiniteLineToBounds(lineAnchor, lineDirUnit, region.bounds,
                                kBBoxClipPaddingMeters, segA, segB)) {
    return out;  // sweep line misses the polygon's bbox entirely
  }
  if (std::hypot((segB - segA).x(), (segB - segA).y()) < kStripeMinLengthMeters) {
    return out;
  }

  // 3. Intersect the bbox-clipped segment with the polygon (multi-piece +
  //    holes). Boost returns one linestring per "enter -> exit" portion.
  BgLinestring lineLs;
  lineLs.emplace_back(segA.x(), segA.y());
  lineLs.emplace_back(segB.x(), segB.y());

  std::vector<BgLinestring> pieces;
  bg::intersection(lineLs, region.polygon, pieces);

  // Identify Clipper outer paths so we can tag each stripe with the actual
  // owning ring (used downstream by the merge stage as a "same-path" hint).
  // Boost can produce more than one piece per outer ring on concave shapes.
  std::vector<int> outerPathIdx;
  outerPathIdx.reserve(region.paths.size());
  for (int i = 0; i < static_cast<int>(region.paths.size()); ++i) {
    if (region.paths[i].size() < 3) continue;
    if (ClipperLib::Area(region.paths[i]) > 0.0) outerPathIdx.push_back(i);
  }

  for (const auto& piece : pieces) {
    if (piece.size() < 2) continue;
    const auto& q0 = piece.front();
    const auto& q1 = piece.back();
    StripeSegment s;
    s.a = QPointF(q0.x(), q0.y());
    s.b = QPointF(q1.x(), q1.y());
    s.mid = (s.a + s.b) * 0.5;
    s.d = dValue;
    s.row = rowIndex;

    // Reject pieces whose midpoint sits outside the multi-polygon. With
    // boost::intersection on a (possibly invalid) multi-polygon residue this
    // can happen on degenerate edges/spikes; better to drop than to ship a
    // stripe that escapes the working area in the visualization and the
    // graph builder.
    if (!bg::covered_by(BgPoint(s.mid.x(), s.mid.y()), region.polygon)) continue;

    // Pick the outer-ring index that contains the midpoint (Clipper-side).
    s.pathIndex = region.outerPathIndex;
    const ClipperLib::IntPoint probe = ClipperUtils::toClip(s.mid);
    for (int oi : outerPathIdx) {
      if (ClipperLib::PointInPolygon(probe, region.paths[oi]) != 0) {
        s.pathIndex = oi;
        break;
      }
    }

    if (std::hypot((s.b - s.a).x(), (s.b - s.a).y()) < kStripeMinLengthMeters) continue;
    out.push_back(s);
  }

  std::sort(out.begin(), out.end(), [&](const StripeSegment& x, const StripeSegment& y) {
    return QPointF::dotProduct(x.mid, lineDirUnit) < QPointF::dotProduct(y.mid, lineDirUnit);
  });
  return out;
}

namespace {
constexpr double kExclusionTouchStepFactor = 0.35;
constexpr double kExclusionTouchMinMeters = 0.45;

double pointToPathDistance(const ClipperLib::Path& path, const QPointF& p) {
  if (path.size() < 2) return std::numeric_limits<double>::max();
  double best = std::numeric_limits<double>::max();
  const int n = static_cast<int>(path.size());
  for (int i = 0; i < n; ++i) {
    const QPointF a = ClipperUtils::fromClip(path[i]);
    const QPointF b = ClipperUtils::fromClip(path[(i + 1) % n]);
    const QPointF ab = b - a;
    const double ab2 = QPointF::dotProduct(ab, ab);
    if (ab2 < kGeomEps) continue;
    const double t = std::clamp(QPointF::dotProduct(p - a, ab) / ab2, 0.0, 1.0);
    const QPointF proj = a + ab * t;
    best = std::min(best, std::hypot((proj - p).x(), (proj - p).y()));
  }
  return best;
}

bool isConcaveVertexCCW(const ClipperLib::Path& path, int vertexIndex) {
  const int n = static_cast<int>(path.size());
  if (n < 3) return false;
  const QPointF prev = ClipperUtils::fromClip(path[(vertexIndex - 1 + n) % n]);
  const QPointF cur = ClipperUtils::fromClip(path[vertexIndex]);
  const QPointF next = ClipperUtils::fromClip(path[(vertexIndex + 1) % n]);
  const QPointF e1 = cur - prev;
  const QPointF e2 = next - cur;
  return (e1.x() * e2.y() - e1.y() * e2.x()) < -kGeomEps;
}

bool pointNearHoleRings(const BgMultiPolygon& mp, const QPointF& p, double eps) {
  for (const auto& poly : mp) {
    for (const auto& inner : poly.inners()) {
      if (inner.size() < 4) continue;
      ClipperLib::Path hole;
      hole.reserve(inner.size());
      for (const auto& bp : inner) {
        hole.push_back(ClipperUtils::toClip(QPointF(bp.x(), bp.y())));
      }
      if (pointToPathDistance(hole, p) <= eps) return true;
    }
  }
  return false;
}

bool pointNearConcaveOuterBoundary(const RegionGeometry& region, const QPointF& p, double eps) {
  for (int pi = 0; pi < static_cast<int>(region.paths.size()); ++pi) {
    const ClipperLib::Path& path = region.paths[static_cast<size_t>(pi)];
    if (path.size() < 3 || ClipperLib::Area(path) <= 0.0) continue;
    const double dist = pointToPathDistance(path, p);
    if (dist > eps) continue;
    const BoundaryAnchor anchor = nearestBoundaryAnchor(path, p, pi);
    if (anchor.segIndex < 0) continue;
    const int n = static_cast<int>(path.size());
    const int v0 = anchor.segIndex;
    const int v1 = (anchor.segIndex + 1) % n;
    if (isConcaveVertexCCW(path, v0) || isConcaveVertexCCW(path, v1)) return true;
    if (anchor.t <= 0.08 && isConcaveVertexCCW(path, v0)) return true;
    if (anchor.t >= 0.92 && isConcaveVertexCCW(path, v1)) return true;
  }
  return false;
}
}  // namespace

bool stripeTouchesExclusionBoundary(const RegionGeometry& insetGeo,
                                    const RegionGeometry& workingGeo,
                                    const StripeSegment& seg, double stepMeters) {
  const double eps =
      std::max(kExclusionTouchMinMeters, stepMeters * kExclusionTouchStepFactor);
  const QPointF samples[3] = {seg.a, seg.b, seg.mid};
  for (const QPointF& p : samples) {
    if (pointNearHoleRings(workingGeo.polygon, p, eps)) return true;
    if (pointNearHoleRings(insetGeo.polygon, p, eps)) return true;
    if (pointNearConcaveOuterBoundary(insetGeo, p, eps)) return true;
  }
  return false;
}

void addAreaOutlines(QGVMap* map, QGVLayer* layer, const ClipperLib::Paths& paths, const QPen& pen) {
  if (!map || !layer) return;
  for (const auto& path : paths) {
    if (path.size() < 3) continue;
    QVector<QGV::GeoPos> geo;
    geo.reserve(static_cast<int>(path.size()) + 1);
    for (const auto& p : path) {
      geo.push_back(map->getProjection()->projToGeo(ClipperUtils::fromClip(p)));
    }
    if (!geo.isEmpty() && geo.front() != geo.back()) {
      geo.push_back(geo.front());
    }
    auto* poly = new GeoPolyline(map);
    poly->setPen(pen);
    poly->points = geo;
    layer->addItem(poly);
  }
}

}  // namespace RouteAlgo
