#include "ClipperUtils.h"

#include <algorithm>

namespace ClipperUtils {
ClipperLib::IntPoint toClip(const QPointF& p) {
  return ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(p.x() * kClipperScale),
                              static_cast<ClipperLib::cInt>(p.y() * kClipperScale));
}

QPointF fromClip(const ClipperLib::IntPoint& p) {
  return QPointF(p.X / kClipperScale, p.Y / kClipperScale);
}

ClipperLib::Path toClipPath(const QVector<QPointF>& points, bool closed) {
  ClipperLib::Path path;
  for (const QPointF& pt : points) {
    path.push_back(toClip(pt));
  }
  if (closed && path.size() > 1 && path.front() != path.back()) {
    path.push_back(path.front());
  }
  return path;
}

QVector<QPointF> fromClipPath(const ClipperLib::Path& path, bool closed) {
  QVector<QPointF> points;
  points.reserve(static_cast<int>(path.size()));
  for (const auto& p : path) {
    points.push_back(fromClip(p));
  }
  if (closed && points.size() > 1 && points.front() != points.back()) {
    points.push_back(points.front());
  }
  return points;
}

ClipperLib::Paths intersection(const ClipperLib::Path& subject,
                               const ClipperLib::Path& clip,
                               bool subjectClosed) {
  ClipperLib::Clipper clp;
  clp.AddPath(subject, ClipperLib::ptSubject, subjectClosed);
  clp.AddPath(clip, ClipperLib::ptClip, true);

  ClipperLib::PolyTree tree;
  clp.Execute(ClipperLib::ctIntersection, tree, ClipperLib::pftNonZero,
              ClipperLib::pftNonZero);

  ClipperLib::Paths result;
  ClipperLib::OpenPathsFromPolyTree(tree, result);
  if (!subjectClosed && !result.empty()) {
    return result;
  }

  result.clear();
  ClipperLib::ClosedPathsFromPolyTree(tree, result);
  return result;
}

ClipperLib::Paths difference(const ClipperLib::Paths& subject,
                             const ClipperLib::Paths& clip) {
  ClipperLib::Clipper clp;
  clp.AddPaths(subject, ClipperLib::ptSubject, true);
  clp.AddPaths(clip, ClipperLib::ptClip, true);

  ClipperLib::Paths result;
  clp.Execute(ClipperLib::ctDifference, result, ClipperLib::pftNonZero,
              ClipperLib::pftNonZero);
  return result;
}

ClipperLib::Paths unionPaths(const ClipperLib::Paths& paths) {
  if (paths.empty()) {
    return {};
  }

  ClipperLib::Clipper clp;
  clp.AddPaths(paths, ClipperLib::ptSubject, true);
  ClipperLib::Paths result;
  clp.Execute(ClipperLib::ctUnion, result, ClipperLib::pftNonZero,
              ClipperLib::pftNonZero);
  return result;
}

ClipperLib::Paths offset(const ClipperLib::Path& path, double deltaMeters,
                         ClipperLib::JoinType joinType) {
  ClipperLib::ClipperOffset co;
  co.AddPath(path, joinType, ClipperLib::etClosedPolygon);
  ClipperLib::Paths out;
  co.Execute(out, deltaMeters * kClipperScale);
  return out;
}

void orientClipperPathOuter(ClipperLib::Path& path) {
  if (path.size() < 3) return;
  if (ClipperLib::Area(path) < 0.0)
    std::reverse(path.begin(), path.end());
}

void orientClipperPathsOuter(ClipperLib::Paths& paths) {
  for (ClipperLib::Path& p : paths)
    orientClipperPathOuter(p);
}
}  // namespace ClipperUtils
