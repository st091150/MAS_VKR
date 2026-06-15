#include "RouteCoveragePlanner.h"

#include <QGeoView/QGVMap.h>
#include <QGeoView/QGVProjection.h>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kEps = 1e-6;

QPointF normalized(const QPointF& v) {
  const double len = std::hypot(v.x(), v.y());
  if (len < kEps) {
    return {0.0, 0.0};
  }
  return {v.x() / len, v.y() / len};
}

double dot(const QPointF& a, const QPointF& b) {
  return a.x() * b.x() + a.y() * b.y();
}

double cross(const QPointF& a, const QPointF& b) {
  return a.x() * b.y() - a.y() * b.x();
}

QPointF closestPointOnPath(const ClipperLib::Path& path, const QPointF& p) {
  if (path.empty()) {
    return p;
  }

  QPointF best = ClipperUtils::fromClip(path.front());
  double bestDist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path.size(); ++i) {
    const QPointF a = ClipperUtils::fromClip(path[i]);
    const QPointF b = ClipperUtils::fromClip(path[(i + 1) % path.size()]);
    const QPointF ab = b - a;
    const double len2 = dot(ab, ab);
    if (len2 < kEps) {
      continue;
    }
    const double t = std::clamp(dot(p - a, ab) / len2, 0.0, 1.0);
    const QPointF q = a + ab * t;
    const double d = std::hypot((q - p).x(), (q - p).y());
    if (d < bestDist) {
      bestDist = d;
      best = q;
    }
  }
  return best;
}

QVector<QPointF> contourToProj(const QVector<QGV::GeoPos>& points, QGVMap* map) {
  QVector<QPointF> out;
  if (!map || !map->getProjection()) {
    return out;
  }

  out.reserve(points.size());
  for (const auto& p : points) {
    out.push_back(map->getProjection()->geoToProj(p));
  }
  return out;
}

QPointF moveAlongClosedPath(const QVector<QPointF>& polygon, const QPointF& startPt,
                            double stepMeters, bool forward) {
  if (polygon.size() < 2 || stepMeters <= 0.0) {
    return startPt;
  }

  double minDist = std::numeric_limits<double>::max();
  int segIndex = 0;
  double tOnSeg = 0.0;
  for (int i = 0; i < polygon.size(); ++i) {
    const QPointF p1 = polygon[i];
    const QPointF p2 = polygon[(i + 1) % polygon.size()];
    const QPointF seg = p2 - p1;
    const double len2 = dot(seg, seg);
    if (len2 < kEps) {
      continue;
    }
    const double t = std::clamp(dot(startPt - p1, seg) / len2, 0.0, 1.0);
    const QPointF proj = p1 + seg * t;
    const double d = std::hypot((proj - startPt).x(), (proj - startPt).y());
    if (d < minDist) {
      minDist = d;
      segIndex = i;
      tOnSeg = t;
    }
  }

  double remaining = stepMeters;
  int current = segIndex;
  double t = tOnSeg;
  while (remaining > kEps) {
    const QPointF p1 = polygon[current];
    const QPointF p2 = polygon[(current + 1) % polygon.size()];
    const QPointF seg = p2 - p1;
    const double len = std::hypot(seg.x(), seg.y());
    if (len < kEps) {
      current = forward ? (current + 1) % polygon.size()
                        : (current - 1 + polygon.size()) % polygon.size();
      t = forward ? 0.0 : 1.0;
      continue;
    }

    const double onSeg = forward ? (1.0 - t) * len : t * len;
    if (remaining <= onSeg + kEps) {
      const double ratio = forward ? t + remaining / len : t - remaining / len;
      return p1 + seg * ratio;
    }
    remaining -= onSeg;
    current = forward ? (current + 1) % polygon.size()
                      : (current - 1 + polygon.size()) % polygon.size();
    t = forward ? 0.0 : 1.0;
  }
  return startPt;
}

bool raySegmentIntersection(const QPointF& origin, const QPointF& rayDir,
                            const QPointF& a, const QPointF& b, double& outRayT) {
  const QPointF edge = b - a;
  const QPointF ao = a - origin;
  const double det = cross(rayDir, edge);
  if (std::abs(det) < kEps) {
    return false;
  }
  const double tRay = cross(ao, edge) / det;
  const double uSeg = cross(ao, rayDir) / det;
  if (tRay <= kEps || uSeg < -kEps || uSeg > 1.0 + kEps) {
    return false;
  }
  outRayT = tRay;
  return true;
}
}  // namespace

RouteCoveragePlanner::Result RouteCoveragePlanner::build(const Input& input, QGVMap* map) {
  Result result;
  if (!map || input.contourGeo.size() < 4 || input.stepMeters <= 0.0) {
    return result;
  }

  const QVector<QPointF> contourProj = contourToProj(input.contourGeo, map);
  ClipperLib::Path insetPolygon = toInsetPolygon(contourProj, input.boundaryOffsetMeters);
  if (insetPolygon.empty()) {
    return result;
  }

  const QPointF startProj = map->getProjection()->geoToProj(input.startPoint);
  QVector<QPointF> routeProj;
  QVector<LocalSegment> segments = buildSequentialSnakeSegments(
      insetPolygon, startProj, input.angleDegrees, input.stepMeters, input.rightSide,
      input.offsetCutMeters, routeProj);
  if (segments.isEmpty()) {
    return result;
  }

  if (routeProj.size() < 2) {
    return result;
  }

  for (const QPointF& p : routeProj) {
    result.routePathGeo.push_back(map->getProjection()->projToGeo(p));
  }

  const ClipperLib::Paths covered = buildCoveredPolygons(segments, input.stepMeters);
  const ClipperLib::Paths insetAsPaths{insetPolygon};
  const ClipperLib::Paths residual = ClipperUtils::difference(insetAsPaths, covered);

  result.coveredPolygonsGeo = convertPathsToGeo(covered, map);
  result.residualPolygonsGeo = convertPathsToGeo(residual, map);
  result.ok = !result.routePathGeo.isEmpty();
  return result;
}

ClipperLib::Path RouteCoveragePlanner::toInsetPolygon(const QVector<QPointF>& contourProj,
                                                      double boundaryOffsetMeters) {
  if (contourProj.size() < 4) {
    return {};
  }

  ClipperLib::Path outer = ClipperUtils::toClipPath(contourProj, true);
  ClipperLib::Paths inset = ClipperUtils::offset(outer, -boundaryOffsetMeters);
  if (inset.empty()) {
    return {};
  }

  auto largest = inset.front();
  double maxArea = std::abs(ClipperLib::Area(largest));
  for (const auto& p : inset) {
    const double area = std::abs(ClipperLib::Area(p));
    if (area > maxArea) {
      maxArea = area;
      largest = p;
    }
  }
  return largest;
}

QVector<RouteCoveragePlanner::LocalSegment> RouteCoveragePlanner::buildSequentialSnakeSegments(
    const ClipperLib::Path& insetPolygon, const QPointF& startPointProj, double angleDegrees,
    double stepMeters, bool rightSide, double offsetCutMeters, QVector<QPointF>& routePathProj) {
  QVector<LocalSegment> segments;
  routePathProj.clear();
  if (insetPolygon.size() < 4 || stepMeters <= 0.0 || offsetCutMeters < 0.0) {
    return segments;
  }

  const double angleRad = qDegreesToRadians(angleDegrees);
  const QPointF dir = normalized({std::cos(angleRad), std::sin(angleRad)});
  if (std::hypot(dir.x(), dir.y()) < kEps) {
    return segments;
  }

  QVector<QPointF> polygon;
  polygon.reserve(static_cast<int>(insetPolygon.size()));
  for (const auto& cp : insetPolygon) {
    polygon.push_back(ClipperUtils::fromClip(cp));
  }
  if (polygon.size() > 1 &&
      std::hypot((polygon.front() - polygon.back()).x(),
                 (polygon.front() - polygon.back()).y()) < kEps) {
    polygon.pop_back();
  }
  if (polygon.size() < 3) {
    return segments;
  }

  QPointF currentStart = closestPointOnPath(insetPolygon, startPointProj);
  bool alongForward = !rightSide;
  QPointF travelDir = dir;

  const int maxIterations = 4000;
  for (int iter = 0; iter < maxIterations; ++iter) {
    auto nearestHit = [&](const QPointF& rayDirection) {
      double bestT = std::numeric_limits<double>::max();
      for (int i = 0; i < polygon.size(); ++i) {
        const QPointF a = polygon[i];
        const QPointF b = polygon[(i + 1) % polygon.size()];
        double tRay = 0.0;
        if (raySegmentIntersection(currentStart, rayDirection, a, b, tRay)) {
          if (tRay < bestT) {
            bestT = tRay;
          }
        }
      }
      return bestT;
    };

    const double tForward = nearestHit(dir);
    const double tBackward = nearestHit(-dir);
    if (tForward == std::numeric_limits<double>::max() &&
        tBackward == std::numeric_limits<double>::max()) {
      break;
    }

    // Pick the longer valid in-polygon reach from boundary point to opposite side.
    if (tForward == std::numeric_limits<double>::max()) {
      travelDir = -dir;
    } else if (tBackward == std::numeric_limits<double>::max()) {
      travelDir = dir;
    } else {
      travelDir = (tForward >= tBackward) ? dir : -dir;
    }

    const double bestT = (travelDir == dir) ? tForward : tBackward;
    QPointF segA = currentStart;
    QPointF segB = currentStart + travelDir * bestT;
    const QPointF segVec = segB - segA;
    const double len = std::hypot(segVec.x(), segVec.y());
    if (len <= offsetCutMeters * 2.0 + kEps) {
      break;
    }
    const QPointF unit = segVec / len;
    segA += unit * offsetCutMeters;
    segB -= unit * offsetCutMeters;
    if (std::hypot((segB - segA).x(), (segB - segA).y()) < kEps) {
      break;
    }
    const QPointF mid = (segA + segB) * 0.5;
    if (ClipperLib::PointInPolygon(ClipperUtils::toClip(mid), insetPolygon) == 0) {
      break;
    }

    if (routePathProj.isEmpty()) {
      routePathProj.push_back(segA);
    } else if (std::hypot((routePathProj.last() - segA).x(), (routePathProj.last() - segA).y()) >
               kEps) {
      routePathProj.push_back(segA);
    }

    routePathProj.push_back(segB);
    segments.push_back(LocalSegment{segA, segB, static_cast<double>(iter), dot(mid, dir)});

    QPointF nextStart = moveAlongClosedPath(polygon, segB, stepMeters, alongForward);
    if (std::hypot((nextStart - segB).x(), (nextStart - segB).y()) < stepMeters * 0.2) {
      break;
    }

    routePathProj.push_back(nextStart);
    currentStart = nextStart;
    travelDir = -travelDir;
  }
  return segments;
}

ClipperLib::Paths RouteCoveragePlanner::buildCoveredPolygons(
    const QVector<LocalSegment>& segments, double stepMeters) {
  ClipperLib::Paths bands;
  const double half = stepMeters * 0.5;
  for (const LocalSegment& seg : segments) {
    const QPointF dir = normalized(seg.b - seg.a);
    const QPointF n(-dir.y(), dir.x());
    QVector<QPointF> rect = {
        seg.a + n * half, seg.b + n * half, seg.b - n * half, seg.a - n * half};
    bands.push_back(ClipperUtils::toClipPath(rect, true));
  }
  return ClipperUtils::unionPaths(bands);
}

QVector<QVector<QGV::GeoPos>> RouteCoveragePlanner::convertPathsToGeo(
    const ClipperLib::Paths& paths, QGVMap* map) {
  QVector<QVector<QGV::GeoPos>> out;
  if (!map || !map->getProjection()) {
    return out;
  }

  for (const auto& path : paths) {
    if (path.size() < 3) {
      continue;
    }
    QVector<QGV::GeoPos> polygonGeo;
    polygonGeo.reserve(static_cast<int>(path.size()) + 1);
    for (const auto& p : path) {
      polygonGeo.push_back(map->getProjection()->projToGeo(ClipperUtils::fromClip(p)));
    }
    if (!polygonGeo.isEmpty() && polygonGeo.front() != polygonGeo.back()) {
      polygonGeo.push_back(polygonGeo.front());
    }
    out.push_back(polygonGeo);
  }
  return out;
}
