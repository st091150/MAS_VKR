#include "ContourPreviewDialog.h"
#include <QPainter>
#include <QPen>

ContourPreviewDialog::ContourPreviewDialog(
    const QVector<QGV::GeoPos>& points,
    const QVector<QGV::GeoPos>& intersections, QWidget* parent)
    : QDialog(parent), m_points(points), m_intersections(intersections) {
  setWindowTitle("Контур имеет пересечения");
  resize(600, 600);
}

void ContourPreviewDialog::paintEvent(QPaintEvent* /*event*/) {
  if (m_points.isEmpty())
    return;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  double minLat = m_points[0].latitude(), maxLat = m_points[0].latitude();
  double minLon = m_points[0].longitude(), maxLon = m_points[0].longitude();
  for (const auto& p : std::as_const(m_points)) {
    if (p.latitude() < minLat)
      minLat = p.latitude();
    if (p.latitude() > maxLat)
      maxLat = p.latitude();
    if (p.longitude() < minLon)
      minLon = p.longitude();
    if (p.longitude() > maxLon)
      maxLon = p.longitude();
  }

  auto mapX = [&](double lon) {
    return (lon - minLon) / (maxLon - minLon) * width();
  };
  auto mapY = [&](double lat) {
    return height() - (lat - minLat) / (maxLat - minLat) * height();
  };

  // Рисуем контур
  QPen pen(Qt::blue, 2);
  painter.setPen(pen);

  for (int i = 0; i < m_points.size() - 1; ++i) {
    painter.drawLine(
        mapX(m_points[i].longitude()), mapY(m_points[i].latitude()),
        mapX(m_points[i + 1].longitude()), mapY(m_points[i + 1].latitude()));
  }

  // Рисуем точки пересечения
  QPen ipen(Qt::red, 6);
  painter.setPen(ipen);
  for (const auto& p : std::as_const(m_intersections)) {
    painter.drawPoint(mapX(p.longitude()), mapY(p.latitude()));
  }
}
