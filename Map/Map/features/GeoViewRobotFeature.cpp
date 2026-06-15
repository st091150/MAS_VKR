#include "GeoViewRobotFeature.h"

#include "../GeoViewWidget.h"

#include <QPixmap>
#include <QTransform>

void GeoViewRobotFeature::redrawCoverageRouteIfPresent(GeoViewWidget& view) {
  if (view.mState.routePoints.size() < 2)
    return;
  view.drawRoute(view.mRouteLayer, view.mState.routePoints, view.mRouteColor, true);
}

void GeoViewRobotFeature::addRobot(GeoViewWidget& view, double latitude, double longitude,
                                   double angle) {
  if (view.mRobotItem.item) {
    view.clearRobotLayer();
    if (!view.mState.routePoints.isEmpty()) {
      view.mState.routePoints.pop_front();
    }
  }

  const auto geoPoint = QGV::GeoPos(latitude, longitude);
  const auto projPos = view.mMap->getProjection()->geoToProj(geoPoint);

  const QPixmap iconPixmap = QPixmap::fromImage(view.mRobotIcon);
  const QPixmap rotated =
      iconPixmap.transformed(QTransform().rotate(angle), Qt::SmoothTransformation);

  auto* item = new QGVIcon();
  item->setGeometry(projPos);
  item->loadImage(rotated.toImage());
  item->setZValue(10.0);
  view.mRobotLayer->addItem(item);

  view.mRobotItem.item = item;
  view.mRobotItem.pos = QGV::GeoPos(latitude, longitude);
  view.mRobotItem.angle = angle;

  view.mState.routePoints.prepend(view.mRobotItem.pos);
  redrawCoverageRouteIfPresent(view);
  view.updateInfoList();
}

void GeoViewRobotFeature::updateRobot(GeoViewWidget& view, double latitude, double longitude,
                                      double angle) {
  if (!view.mRobotItem.item) {
    addRobot(view, latitude, longitude, angle);
    return;
  }

  view.mRobotLayer->removeItem(view.mRobotItem.item);
  const auto geoPoint = QGV::GeoPos(latitude, longitude);
  const auto projPos = view.mMap->getProjection()->geoToProj(geoPoint);

  const QPixmap iconPixmap = QPixmap::fromImage(view.mRobotIcon);
  const QPixmap rotated =
      iconPixmap.transformed(QTransform().rotate(angle), Qt::SmoothTransformation);

  auto* item = new QGVIcon();
  item->setGeometry(projPos);
  item->loadImage(rotated.toImage());
  item->setZValue(10.0);
  view.mRobotLayer->addItem(item);

  view.mRobotItem.item = item;
  view.mRobotItem.pos = geoPoint;
  view.mRobotItem.angle = angle;

  if (!view.mState.routePoints.isEmpty()) {
    view.mState.routePoints[0] = view.mRobotItem.pos;
    redrawCoverageRouteIfPresent(view);
  }

  view.updateInfoList();
}
