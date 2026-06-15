#pragma once

#include <QGeoView/QGVDrawItem.h>
#include <QGeoView/QGVProjection.h>
#include <QPainterPath>
#include <QPen>

class GeoPolyline : public QGVDrawItem {
  Q_OBJECT
 public:
  explicit GeoPolyline(QGVMap* map, QObject* parent = nullptr);

  QVector<QGV::GeoPos> points;

  void setPen(const QPen& pen) {
    mPen = pen;
    update();
  }
  QPolygonF calcArrowPolygon(const QPointF& start, const QPointF& end) const;
  bool drawArrowOnEnd = false;

 protected:
  void onProjection(QGVMap* map) override;
  void onUpdate() override;
  QPainterPath projShape() const override;
  void projPaint(QPainter* p) override;

 private:
  QGVMap* mMap = nullptr;
  QVector<QPointF> mProjPoints;
  QPen mPen = QPen(QColor(255, 0, 0, 180), 2);  // красная линия по умолчанию
  QPainterPath mCachedPath;

  double arrowLength = 3.0;
  double arrowWidth = 1.8;

  void rebuild();
  void drawArrow(QPainter* p, const QPointF& start, const QPointF& end);
};
