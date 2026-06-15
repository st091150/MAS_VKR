#pragma once

#include "../../clipper/clipper.hpp"
#include <QPointF>
#include <QVector>
#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

class QPen;
class QGVLayer;
class QGVMap;

namespace RouteAlgo {

namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;  // single outer + holes (CW outer / CCW holes after correct)
using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;

struct StripeSegment {
  QPointF a;
  QPointF b;
  QPointF mid;
  double d = 0.0;
  int row = 0;
  int pathIndex = -1;
};

struct BoundaryAnchor {
  int pathIndex = -1;
  int segIndex = -1;
  double t = 0.0;
  double arc = 0.0;
  QPointF point;
  double distance = 1e100;
};

struct RegionBounds {
  double minX = 1e100;
  double minY = 1e100;
  double maxX = -1e100;
  double maxY = -1e100;
};

// Cached representation of the working area:
//   * one or more disjoint outer boundaries
//   * zero or more holes attached to their containing outer
// The Clipper representation is preserved for boolean ops & boundary indexing.
// The boost multi-polygon is used for fast, holes-aware predicates and is
// always valid even when residual regions split into several disjoint pieces
// (e.g. the leftover spikes of a star polygon after a sweep pass).
struct RegionGeometry {
  ClipperLib::Paths paths;             // original Clipper-side representation
  BgMultiPolygon polygon;              // boost multi-polygon (outers + holes)
  std::vector<double> pathPerimeters;  // perimeter per `paths[i]`
  int outerPathIndex = -1;             // index of largest outer (compat hint for stripe.pathIndex)
  RegionBounds bounds;
};

RegionGeometry buildRegionGeometry(const ClipperLib::Paths& region);

QPointF normalizeOrZero(const QPointF& v);

// Predicates / proximity (boost-backed when working with RegionGeometry).
bool regionContainsPoint(const RegionGeometry& region, const QPointF& p);
bool directConnectorInsideRegion(const RegionGeometry& region, const QPointF& a, const QPointF& b);
QPointF nearestPointOnRegion(const ClipperLib::Paths& region, const QPointF& p);

// Boundary helpers — work on a single closed path; perimeter must be supplied
// (the caller is expected to have it cached in RegionGeometry).
BoundaryAnchor nearestBoundaryAnchor(const ClipperLib::Path& path, const QPointF& p, int pathIndex);
// Closest contour edge over all closed paths in `region` (for merge / touch tests).
BoundaryAnchor nearestBoundaryAnchorOnRegion(const RegionGeometry& region, const QPointF& p);
QVector<QPointF> buildBoundaryConnector(const ClipperLib::Path& path, const BoundaryAnchor& from,
                                        const BoundaryAnchor& to, double cachedPerimeter);
double shortestBoundaryArcDistance(const ClipperLib::Path& path, const BoundaryAnchor& from,
                                   const BoundaryAnchor& to, double cachedPerimeter);

// Bulk geometry helpers.
double regionSignedArea(const ClipperLib::Paths& paths);
RegionBounds computeRegionBounds(const ClipperLib::Paths& region);
ClipperLib::Paths intersectRegionWithBand(const ClipperLib::Paths& region, const ClipperLib::Path& band);

// Line ∩ polygon (with holes). Uses boost::geometry::intersection internally.
std::vector<StripeSegment> intersectInfiniteLineWithRegion(const RegionGeometry& region,
                                                           const QPointF& linePoint,
                                                           const QPointF& lineDirUnit, double dValue,
                                                           int rowIndex);

// True when a stripe endpoint lies near a user cutout / hole ring or near a
// concave pocket of the outer boundary (inset). Such stripes must not bridge
// to interior stripes in the graph or merge with distant components.
bool stripeTouchesExclusionBoundary(const RegionGeometry& insetGeo,
                                    const RegionGeometry& workingGeo,
                                    const StripeSegment& seg, double stepMeters);

void addAreaOutlines(QGVMap* map, QGVLayer* layer, const ClipperLib::Paths& paths, const QPen& pen);

}  // namespace RouteAlgo
