#pragma once

#include "../utils/ClipperUtils.h"
#include <QGeoView/QGVMap.h>

#include <QPointF>
#include <QVector>

class RouteCoveragePlanner {
 public:
  struct Input {
    QVector<QGV::GeoPos> contourGeo;
    QGV::GeoPos startPoint;
    QGV::GeoPos endPoint;
    double angleDegrees = 0.0;
    double stepMeters = 4.0;
    double boundaryOffsetMeters = 2.0;
    bool rightSide = false;
    double offsetCutMeters = 2.0;
  };

  struct Result {
    QVector<QGV::GeoPos> routePathGeo;
    QVector<QVector<QGV::GeoPos>> coveredPolygonsGeo;
    QVector<QVector<QGV::GeoPos>> residualPolygonsGeo;
    bool ok = false;
  };

  static Result build(const Input& input, QGVMap* map);

 private:
  struct LocalSegment {
    QPointF a;
    QPointF b;
    double stripeProj = 0.0;
    double alongMid = 0.0;
  };

  static ClipperLib::Path toInsetPolygon(const QVector<QPointF>& contourProj,
                                         double boundaryOffsetMeters);
  static QVector<LocalSegment> buildSequentialSnakeSegments(
      const ClipperLib::Path& insetPolygon, const QPointF& startPointProj,
      double angleDegrees, double stepMeters, bool rightSide, double offsetCutMeters,
      QVector<QPointF>& routePathProj);
  static ClipperLib::Paths buildCoveredPolygons(const QVector<LocalSegment>& segments,
                                                double stepMeters);
  static QVector<QVector<QGV::GeoPos>> convertPathsToGeo(const ClipperLib::Paths& paths,
                                                         QGVMap* map);
};
