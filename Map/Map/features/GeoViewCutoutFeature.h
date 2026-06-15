#pragma once
 
class GeoViewWidget;
namespace QGV {
class GeoPos;
}
 
// Логика вырезов (cutout) вынесена из GeoViewWidget для упрощения виджета.
class GeoViewCutoutFeature {
 public:
  static void startCutoutPolygonMode(GeoViewWidget& view);
  static void handleCutoutPolygonClick(GeoViewWidget& view, const QGV::GeoPos& pos);
  static bool finishCutoutPolygon(GeoViewWidget& view);
  static void undoLastCutout(GeoViewWidget& view);
 
  static void redrawCutouts(GeoViewWidget& view);
  static void redrawActiveCutout(GeoViewWidget& view);
  static void clearCutouts(GeoViewWidget& view);
};
