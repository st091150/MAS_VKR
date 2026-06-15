#include "GeoViewInteractionFeature.h"
 
#include "../GeoViewWidget.h"
 
#include <QCursor>
#include <QToolTip>
 
void GeoViewInteractionFeature::onMapMousePress(GeoViewWidget& view, QPointF projPos) {
  const QGV::GeoPos geoPos = view.mMap->getProjection()->projToGeo(projPos);
 
  if (view.mInteractionMode == GeoViewWidget::InteractionMode::CutoutPolygon) {
    view.handleCutoutPolygonClick(geoPos);
    return;
  }

  if (view.mInteractionMode == GeoViewWidget::InteractionMode::DrawContour) {
    view.handleDrawContourClick(geoPos);
    return;
  }
 
  if (view.mInteractionMode == GeoViewWidget::InteractionMode::PlaceRobot) {
    if (view.mRobotItem.item)
      view.updateRobot(geoPos.latitude(), geoPos.longitude(), view.mRobotItem.angle);
    else
      view.addRobot(geoPos.latitude(), geoPos.longitude(), 0.0);
    view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
    QToolTip::showText(QCursor::pos(), QObject::tr("Позиция робота задана"), view.mMap);
    view.setUiStatus(QObject::tr("Позиция робота на карте задана"),
                     GeoViewWidget::UiStatusLevel::Success);
    return;
  }
 
  if (view.mInteractionMode == GeoViewWidget::InteractionMode::ManualRoute) {
    view.handleMapClick(geoPos);
  }
 
  auto showSelectionHint = [&view](const QString& text) {
    // Non-blocking hint does not break mouse release on map.
    QToolTip::showText(QCursor::pos(), text, view.mMap);
  };
 
  if (view.mInteractionMode == GeoViewWidget::InteractionMode::SelectStartPoint) {
    if (!view.mContour || !view.mContour->isPointOnContour(geoPos)) {
      showSelectionHint(QObject::tr("Стартовая точка должна быть на контуре"));
      view.setUiStatus(QObject::tr("Стартовая точка должна быть выбрана на контуре"),
                       GeoViewWidget::UiStatusLevel::Warning);
      return;
    }
 
    view.mState.manualStartPoint = view.mContour->closestPointOnContour(geoPos);
    showSelectionHint(QObject::tr("Стартовая точка выбрана на контуре"));
    view.setUiStatus(QObject::tr("Стартовая точка выбрана"), GeoViewWidget::UiStatusLevel::Success);
  } else if (view.mInteractionMode == GeoViewWidget::InteractionMode::SelectEndPoint) {
    view.mState.manualEndPoint = geoPos;
    showSelectionHint(QObject::tr("Конечная точка выбрана"));
    view.setUiStatus(QObject::tr("Конечная точка выбрана"), GeoViewWidget::UiStatusLevel::Success);
  }
 
  view.handleMapClickStartRoutePoint(geoPos);
}
