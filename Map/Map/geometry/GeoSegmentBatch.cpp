#include "GeoSegmentBatch.h"

#include <QGeoView/QGVMap.h>
#include <QPainter>

GeoSegmentBatch::GeoSegmentBatch(QGVMap* map, QObject* parent)
    : QGVDrawItem(), mMap(map) {
  Q_UNUSED(parent);
}

void GeoSegmentBatch::onProjection(QGVMap* map) {
  QGVDrawItem::onProjection(map);
  rebuild();
}

void GeoSegmentBatch::onUpdate() {
  rebuild();
}

void GeoSegmentBatch::rebuild() {
  mCachedPath = QPainterPath();
  if (!mMap)
    return;

  auto* proj = mMap->getProjection();
  if (!proj)
    return;

  for (const auto& segment : std::as_const(segments)) {
    const QPointF a = proj->geoToProj(segment.first);
    const QPointF b = proj->geoToProj(segment.second);
    mCachedPath.moveTo(a);
    mCachedPath.lineTo(b);
  }
}

QPainterPath GeoSegmentBatch::projShape() const {
  return mCachedPath;
}

void GeoSegmentBatch::projPaint(QPainter* p) {
  p->setRenderHint(QPainter::Antialiasing, segments.size() <= 100);
  p->setPen(mPen);
  p->drawPath(mCachedPath);
}
