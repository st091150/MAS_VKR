#include "GeoPolyline.h"
#include <QGeoView/QGVMap.h>
#include <QPainter.h>
#include <cmath>

GeoPolyline::GeoPolyline(QGVMap* map, QObject* parent)
    : QGVDrawItem(), mMap(map) {
  Q_UNUSED(parent);
}

void GeoPolyline::onProjection(QGVMap* map) {
  QGVDrawItem::onProjection(map);
  rebuild();
}

void GeoPolyline::onUpdate() {
  rebuild();
}

void GeoPolyline::rebuild() {
  mProjPoints.clear();
  mCachedPath = QPainterPath();

  if (!mMap)
    return;

  auto* proj = mMap->getProjection();
  if (!proj)
    return;

  for (const auto& gp : std::as_const(points))
    mProjPoints.push_back(proj->geoToProj(gp));

  if (!mProjPoints.isEmpty()) {
    mCachedPath.moveTo(mProjPoints.first());
    for (int i = 1; i < mProjPoints.size(); i++)
      mCachedPath.lineTo(mProjPoints[i]);
  }
}

QPolygonF GeoPolyline::calcArrowPolygon(const QPointF& start,
                                        const QPointF& end) const {
  QLineF line(start, end);
  QPointF dir = line.p2() - line.p1();
  double len = std::hypot(dir.x(), dir.y());
  if (len != 0)
    dir /= len;

  double angle = std::atan2(dir.y(), dir.x());

  QPointF tip = end;
  QPointF left =
      tip -
      QPointF(arrowLength * std::cos(angle) - arrowWidth * std::sin(angle),
              arrowLength * std::sin(angle) + arrowWidth * std::cos(angle));
  QPointF right =
      tip -
      QPointF(arrowLength * std::cos(angle) + arrowWidth * std::sin(angle),
              arrowLength * std::sin(angle) - arrowWidth * std::cos(angle));

  QPolygonF arrowHead;
  arrowHead << tip << left << right;
  return arrowHead;
}

QPainterPath GeoPolyline::projShape() const {
  QPainterPath path = mCachedPath;

  if (drawArrowOnEnd && mProjPoints.size() >= 2) {
    int n = mProjPoints.size();
    QPolygonF arrowHead =
        calcArrowPolygon(mProjPoints[n - 2], mProjPoints[n - 1]);
    path.addPolygon(arrowHead);
  }

  return path;
}

void GeoPolyline::projPaint(QPainter* p) {
  if (mProjPoints.size() > 100)
    p->setRenderHint(QPainter::Antialiasing, false);
  else
    p->setRenderHint(QPainter::Antialiasing, true);

  p->setPen(mPen);

  if (!drawArrowOnEnd || mProjPoints.size() < 2) {
    p->drawPath(mCachedPath);
    return;
  }

  QPainterPath path = mCachedPath;

  int n = mProjPoints.size();
  QPointF start = mProjPoints[n - 2];
  QPointF end = mProjPoints[n - 1];

  QLineF lastSegment(start, end);
  QPointF dir = lastSegment.p2() - lastSegment.p1();
  double len = std::hypot(dir.x(), dir.y());

  if (len != 0)
    dir /= len;

  QPointF shortEnd = end - dir * arrowLength;

  QPainterPath partialPath;
  partialPath.moveTo(mProjPoints[0]);
  for (int i = 1; i < n - 1; ++i)
    partialPath.lineTo(mProjPoints[i]);
  partialPath.lineTo(shortEnd);

  p->drawPath(partialPath);

  drawArrow(p, shortEnd, end);
}

void GeoPolyline::drawArrow(QPainter* p, const QPointF& start,
                            const QPointF& end) {

  QColor arrowColor = mPen.color().lighter(110);
  arrowColor.setAlpha(255);
  p->setPen(Qt::NoPen);
  p->setBrush(arrowColor);

  QPolygonF arrowHead = calcArrowPolygon(start, end);
  p->drawPolygon(arrowHead, Qt::WindingFill);
}
