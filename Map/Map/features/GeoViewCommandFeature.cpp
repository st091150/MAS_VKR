#include "GeoViewCommandFeature.h"

#include "../GeoViewWidget.h"
#include "../services/RouteCommandBuilder.h"
#include "../utils/MapLog.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QMessageBox>
#include <QtMath>

namespace {
double haversineDistance(const QGV::GeoPos& start, const QGV::GeoPos& end) {
  const double lat1 = qDegreesToRadians(start.latitude());
  const double lat2 = qDegreesToRadians(end.latitude());
  const double dLat = lat2 - lat1;
  const double dLon = qDegreesToRadians(end.longitude() - start.longitude());

  const double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  const double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_METERS * c;
}

double calculateBearing(const QGV::GeoPos& start, const QGV::GeoPos& end) {
  const double lat1 = qDegreesToRadians(start.latitude());
  const double lat2 = qDegreesToRadians(end.latitude());
  const double dLon = qDegreesToRadians(end.longitude() - start.longitude());

  const double y = sin(dLon) * cos(lat2);
  const double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  double bearingDeg = qRadiansToDegrees(atan2(y, x));
  if (bearingDeg > 180)
    bearingDeg -= 360;
  if (bearingDeg < -180)
    bearingDeg += 360;
  return bearingDeg;
}

QPointF computeGazeboPoint(const QGV::GeoPos& start, const QGV::GeoPos& end) {
  const double distance = haversineDistance(start, end);
  const double bearingRad = qDegreesToRadians(calculateBearing(start, end));
  const double xEast = distance * sin(bearingRad);
  const double yNorth = distance * cos(bearingRad);
  return QPointF(xEast, yNorth);
}
}  // namespace

void GeoViewCommandFeature::createRoute(GeoViewWidget& view) {
  view.clearRouteCommands();

  if (view.mState.routePoints.isEmpty() || view.mRobotItem.item == nullptr) {
    const QString msg = view.tr("Проверьте входные данные:\n"
                                "- укажите начальную позицию робота\n"
                                "- постройте маршрут");
    QMessageBox::warning(&view, view.tr("Невозможно построить команды"), msg);
    view.setUiStatus(view.tr("Команды не созданы: не выполнены предусловия"),
                     GeoViewWidget::UiStatusLevel::Warning);
    return;
  }

  view.mState.routeCommands = RouteCommandBuilder::buildRouteCommands(
      view.mRobotItem.pos, view.mRobotItem.angle, view.mState.routePoints);

  emit view.routeBuilt(*view.mState.routeCommands);
  view.setUiStatus(view.tr("Команды построены: %1").arg(view.mState.routeCommands->array().size()),
                   GeoViewWidget::UiStatusLevel::Success);
  qCDebug(logRoute) << view.tr("Маршрут построен.") << view.mState.routeCommands->array().size();
}

void GeoViewCommandFeature::showRobotCommandsJson(GeoViewWidget& view) {
  if (!view.mInfoWidget || !view.mState.routeCommands.has_value())
    return;

  view.mInfoWidget->clear();
  view.mInfoWidget->addItem(view.mState.routeCommands->toJson(QJsonDocument::Indented));
  view.setUiStatus(view.tr("JSON команд отображен в панели Info"), GeoViewWidget::UiStatusLevel::Info);
}

QJsonDocument GeoViewCommandFeature::getRouteCommands(const GeoViewWidget& view) {
  if (view.mState.routeCommands.has_value())
    return *view.mState.routeCommands;
  return QJsonDocument();
}

QJsonDocument GeoViewCommandFeature::generateGazeboJson(GeoViewWidget& view) {
  if (!view.mRobotItem.item || !view.mState.routeCommands.has_value() || !view.mContour) {
    const QString msg = view.tr("Для экспорта в Gazebo нужно:\n"
                                "- начальное положение робота\n"
                                "- контур\n"
                                "- построенный маршрут и команды");
    QMessageBox::warning(&view, view.tr("Недостаточно данных"), msg);
    view.setUiStatus(view.tr("Экспорт в Gazebo недоступен: не хватает данных"),
                     GeoViewWidget::UiStatusLevel::Warning);
    return QJsonDocument();
  }

  const QGV::GeoPos startPos = view.mRobotItem.pos;
  QJsonArray pointsArray;
  for (auto point = view.mContour->points().begin();
       point != view.mContour->points().end() - 1; ++point) {
    const QPointF gazeboPoint = computeGazeboPoint(startPos, *point);
    QJsonObject pointObj;
    pointObj["x"] = gazeboPoint.y();
    pointObj["y"] = -gazeboPoint.x();
    pointObj["z"] = 0.0;
    pointsArray.append(pointObj);
  }

  QJsonObject contourObj;
  contourObj["points"] = pointsArray;

  QJsonArray newCommands;
  auto normalizeAngleRad = [](double a) {
    while (a > M_PI)
      a -= 2 * M_PI;
    while (a < -M_PI)
      a += 2 * M_PI;
    return a;
  };

  for (const auto& cmdVal : view.mState.routeCommands->array()) {
    QJsonObject cmdObj = cmdVal.toObject();
    if (cmdObj["cmd"].toString() == "rotate") {
      const double deltaDeg = cmdObj["data"].toObject()["delta_angle"].toDouble();
      const double deltaRad = normalizeAngleRad(qDegreesToRadians(deltaDeg));
      QJsonObject data;
      data["delta_angle"] = qRadiansToDegrees(deltaRad);
      cmdObj["data"] = data;
    }
    newCommands.append(cmdObj);
  }

  QJsonObject rootObj;
  rootObj["contour"] = contourObj;
  rootObj["commands"] = newCommands;
  return QJsonDocument(rootObj);
}

namespace {
constexpr double kGazeboWaypointDefaultReachM = 0.9;
}  // namespace

QJsonDocument GeoViewCommandFeature::generateGazeboWaypointsJson(GeoViewWidget& view) {
  if (!view.mRobotItem.item || view.mState.routePoints.isEmpty()) {
    const QString msg =
        view.tr("Для файла waypoints нужны:\n"
                "• начальная позиция робота на карте\n"
                "• построенный маршрут (полилиния)");
    QMessageBox::warning(&view, view.tr("Недостаточно данных"), msg);
    view.setUiStatus(view.tr("Waypoints для Gazebo: нет робота или маршрута"),
                     GeoViewWidget::UiStatusLevel::Warning);
    return QJsonDocument();
  }

  QJsonObject origin;
  origin["latitude"] = view.mRobotItem.pos.latitude();
  origin["longitude"] = view.mRobotItem.pos.longitude();
  origin["yaw"] = qDegreesToRadians(view.mRobotItem.angle);

  QJsonArray waypoints;
  for (int i = 0; i < view.mState.routePoints.size(); ++i) {
    const QGV::GeoPos& p = view.mState.routePoints[static_cast<size_t>(i)];
    QJsonObject wp;
    wp["id"] = QString::number(i + 1);
    wp["latitude"] = p.latitude();
    wp["longitude"] = p.longitude();
    wp["reach_radius_m"] = kGazeboWaypointDefaultReachM;
    waypoints.append(wp);
  }

  QJsonObject root;
  root["origin"] = origin;
  root["default_reach_radius_m"] = kGazeboWaypointDefaultReachM;
  root["waypoints"] = waypoints;
  return QJsonDocument(root);
}
