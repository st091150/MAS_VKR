#pragma once
#include <QGeoView/QGVMap.h>
#include <QVector>
#include "GeoPolyline.h"


class Contour {
 public:
  Contour(QGVMap* map);

  void setPoints(const QVector<QGV::GeoPos>& points);
  const QVector<QGV::GeoPos>& points() const;

  void draw();
  void clear();
  void generateParallelRoute(double spacing);
  bool isPointOnContour(const QGV::GeoPos& point,
                        double toleranceMeters = 1.0) const;
  double distanceToSegment(const QGV::GeoPos& p, const QGV::GeoPos& v,
                           const QGV::GeoPos& w) const;
  QGV::GeoPos closestPointOnContour(const QGV::GeoPos& point) const;
  QGV::GeoPos projectPointToSegment(const QGV::GeoPos& p, const QGV::GeoPos& a,
                                    const QGV::GeoPos& b,
                                    double& outDistanceMeters) const;

 private:
  QGVMap* mMap = nullptr;
  QVector<QGV::GeoPos> mPoints;
  GeoPolyline* mPolyline = nullptr;
};
