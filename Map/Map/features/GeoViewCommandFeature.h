#pragma once

class GeoViewWidget;
class QJsonObject;
class QJsonDocument;

class GeoViewCommandFeature {
 public:
  static void createRoute(GeoViewWidget& view);
  static void showRobotCommandsJson(GeoViewWidget& view);
  static QJsonDocument getRouteCommands(const GeoViewWidget& view);
  static QJsonDocument generateGazeboJson(GeoViewWidget& view);
  /// Geo JSON: origin (lat, lon, yaw rad), default_reach_radius_m, waypoints[].
  static QJsonDocument generateGazeboWaypointsJson(GeoViewWidget& view);
};
