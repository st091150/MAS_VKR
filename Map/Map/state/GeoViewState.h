#pragma once

#include <QGeoView/QGVGlobal.h>
#include <QJsonDocument>
#include <QVector>
#include <optional>

struct GeoViewState {
  QVector<QGV::GeoPos> routePoints;
  std::optional<QJsonDocument> routeCommands;

  QGV::GeoPos manualStartPoint = {0, 0};
  QGV::GeoPos manualEndPoint = {0, 0};

  // Route start/end markers rendered on the map (owned by the map/layer).
  // Stored here to keep route-related state localized.
  QGVItem* startPointMarker = nullptr;
  QGVItem* endPointMarker = nullptr;
};
