#pragma once

#include <optional>
#include <QVector>

namespace QGV {
class GeoPos;
}
class GeoViewWidget;

class GeoViewContourFeature {
 public:
  static std::optional<QGV::GeoPos> segmentIntersection(
      GeoViewWidget& view, const QGV::GeoPos& a, const QGV::GeoPos& b,
      const QGV::GeoPos& c, const QGV::GeoPos& d);
  static QVector<QGV::GeoPos> polygonSelfIntersections(
      GeoViewWidget& view, const QVector<QGV::GeoPos>& points);
  static void addContour(GeoViewWidget& view);
  static void removeContour(GeoViewWidget& view);

  static void startDrawContourMode(GeoViewWidget& view);
  static void handleDrawContourClick(GeoViewWidget& view, const QGV::GeoPos& pos);
  static void cancelDrawContour(GeoViewWidget& view);
  static void redrawActiveContourDraft(GeoViewWidget& view);

 private:
  static void zoomMapToContour(GeoViewWidget& view, const QVector<QGV::GeoPos>& points);
  static bool applyContourPolygon(GeoViewWidget& view, const QVector<QGV::GeoPos>& points);
};
