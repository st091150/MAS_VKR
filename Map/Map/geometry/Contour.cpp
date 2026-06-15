#include "Contour.h"
#include <qvectornd.h>

Contour::Contour(QGVMap* map) : mMap(map) {}

void Contour::setPoints(const QVector<QGV::GeoPos>& points) {
  mPoints = points;
}

const QVector<QGV::GeoPos>& Contour::points() const {
  return mPoints;
}

bool Contour::isPointOnContour(const QGV::GeoPos& point,
                               double toleranceMeters) const {
  if (mMap && mPoints.size() < 2)
    return false;

  for (int i = 0; i < mPoints.size() - 1; ++i) {
    const QGV::GeoPos& p1 = mPoints[i];
    const QGV::GeoPos& p2 = mPoints[i + 1];

    // Вычисляем расстояние от point до сегмента p1-p2
    double distance = distanceToSegment(point, p1, p2);
    if (distance <= toleranceMeters) {
      return true;
    }
  }

  return false;
}

double Contour::distanceToSegment(const QGV::GeoPos& p, const QGV::GeoPos& v,
                                  const QGV::GeoPos& w) const {
  QPointF pp = mMap->getProjection()->geoToProj(p);
  QPointF vp = mMap->getProjection()->geoToProj(v);
  QPointF wp = mMap->getProjection()->geoToProj(w);

  QVector2D vw(wp - vp);
  QVector2D vp_p(pp - vp);

  double len2 = QVector2D::dotProduct(vw, vw);
  if (len2 == 0.0)
    return QLineF(pp, vp).length();

  double t = QVector2D::dotProduct(vp_p, vw) / len2;
  t = qBound(0.0, t, 1.0);

  QPointF projection = vp + t * (wp - vp);
  return QLineF(pp, projection).length();
}

QGV::GeoPos Contour::projectPointToSegment(const QGV::GeoPos& p,
                                           const QGV::GeoPos& v,
                                           const QGV::GeoPos& w,
                                           double& outDistanceMeters) const {
  QPointF pp = mMap->getProjection()->geoToProj(p);
  QPointF vp = mMap->getProjection()->geoToProj(v);
  QPointF wp = mMap->getProjection()->geoToProj(w);

  QVector2D vw(wp - vp);
  QVector2D vp_p(pp - vp);

  double len2 = QVector2D::dotProduct(vw, vw);
  if (len2 == 0.0) {
    outDistanceMeters = QLineF(pp, vp).length();
    return v;
  }

  double t = QVector2D::dotProduct(vp_p, vw) / len2;
  t = qBound(0.0, t, 1.0);

  QPointF proj = vp + t * (wp - vp);
  outDistanceMeters = QLineF(pp, proj).length();

  return mMap->getProjection()->projToGeo(proj);
}

QGV::GeoPos Contour::closestPointOnContour(const QGV::GeoPos& point) const {
  QGV::GeoPos closest = mPoints.first();
  double minDistance = std::numeric_limits<double>::max();

  for (int i = 0; i < mPoints.size() - 1; ++i) {
    double dist = 0.0;
    QGV::GeoPos proj =
        projectPointToSegment(point, mPoints[i], mPoints[i + 1], dist);

    if (dist < minDistance) {
      minDistance = dist;
      closest = proj;
    }
  }

  return closest;
}

void Contour::draw() {
  if (!mMap || mPoints.isEmpty())
    return;

  if (!mPolyline) {
    mPolyline = new GeoPolyline(mMap);
    mPolyline->points = mPoints;
    mMap->addItem(mPolyline);
  }

  mPolyline->points = mPoints;
}

void Contour::clear() {
  if (mPolyline && mMap) {
    mMap->removeItem(mPolyline);
    delete mPolyline;
    mPolyline = nullptr;
  }
  mPoints.clear();
}
