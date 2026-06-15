#pragma once

class GeoViewWidget;

class GeoViewRobotFeature {
 public:
  static void addRobot(GeoViewWidget& view, double latitude, double longitude, double angle);
  static void updateRobot(GeoViewWidget& view, double latitude, double longitude, double angle);

 private:
  static void redrawCoverageRouteIfPresent(GeoViewWidget& view);
};
