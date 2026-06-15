#pragma once

#include <QGeoView/QGVDrawItem.h>
#include <QGeoView/QGVProjection.h>
#include <QPainterPath>
#include <QPen>
#include <QVector>

class GeoSegmentBatch : public QGVDrawItem {
 public:
  explicit GeoSegmentBatch(QGVMap* map, QObject* parent = nullptr);

  QVector<QPair<QGV::GeoPos, QGV::GeoPos>> segments;

  void setPen(const QPen& pen) {
    mPen = pen;
    update();
  }

 protected:
  void onProjection(QGVMap* map) override;
  void onUpdate() override;
  QPainterPath projShape() const override;
  void projPaint(QPainter* p) override;

 private:
  QGVMap* mMap = nullptr;
  QPen mPen = QPen(QColor(255, 255, 255, 160), 1);
  QPainterPath mCachedPath;

  void rebuild();
};
