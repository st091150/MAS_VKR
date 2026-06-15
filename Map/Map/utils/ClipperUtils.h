#pragma once

#include "clipper/clipper.hpp"
#include <QPointF>
#include <QVector>

namespace ClipperUtils {
constexpr double kClipperScale = 1000.0;
ClipperLib::IntPoint toClip(const QPointF& p);
QPointF fromClip(const ClipperLib::IntPoint& p);
ClipperLib::Path toClipPath(const QVector<QPointF>& points, bool closed = true);
QVector<QPointF> fromClipPath(const ClipperLib::Path& path, bool closed = true);
ClipperLib::Paths intersection(const ClipperLib::Path& subject,
                               const ClipperLib::Path& clip,
                               bool subjectClosed = true);
ClipperLib::Paths difference(const ClipperLib::Paths& subject,
                             const ClipperLib::Paths& clip);
ClipperLib::Paths unionPaths(const ClipperLib::Paths& paths);
ClipperLib::Paths offset(const ClipperLib::Path& path, double deltaMeters,
                         ClipperLib::JoinType joinType = ClipperLib::jtMiter);

// Clipper boolean ops use pftNonZero: overlapping clips with opposite winding
// cancel inside the overlap. Force counter-clockwise (positive Clipper area)
// so union(difference) of cutouts matches the filled regions drawn on the map.
void orientClipperPathOuter(ClipperLib::Path& path);
void orientClipperPathsOuter(ClipperLib::Paths& paths);
}  // namespace ClipperUtils
