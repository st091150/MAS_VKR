#include "RouteCommandBuilder.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QtMath>

namespace {
double normalizeAngleDegrees(double angle) {
  while (angle > 180.0) {
    angle -= 360.0;
  }
  while (angle < -180.0) {
    angle += 360.0;
  }
  return angle;
}
}  // namespace

QJsonDocument RouteCommandBuilder::buildRouteCommands(
    const QGV::GeoPos& startPos, double startAngleDegrees,
    const QVector<QGV::GeoPos>& routePoints) {
  QJsonArray commands;
  QGV::GeoPos prevPos = startPos;
  double prevAngle = startAngleDegrees;  // 0° = north
  constexpr double earthRadiusMeters = 6371000.0;

  for (const auto& nextPos : routePoints) {
    const double lat1 = qDegreesToRadians(prevPos.latitude());
    const double lon1 = qDegreesToRadians(prevPos.longitude());
    const double lat2 = qDegreesToRadians(nextPos.latitude());
    const double lon2 = qDegreesToRadians(nextPos.longitude());

    const double dLat = lat2 - lat1;
    const double dLon = lon2 - lon1;

    const double a = qSin(dLat / 2) * qSin(dLat / 2) +
                     qCos(lat1) * qCos(lat2) * qSin(dLon / 2) * qSin(dLon / 2);
    const double c = 2 * qAtan2(qSqrt(a), qSqrt(1 - a));
    const double distance = earthRadiusMeters * c;

    // Ignore short segments (<20 cm) to reduce noise.
    if (distance < 0.2) {
      prevPos = nextPos;
      continue;
    }

    const double x = qSin(dLon) * qCos(lat2);
    const double y = qCos(lat1) * qSin(lat2) - qSin(lat1) * qCos(lat2) * qCos(dLon);
    double azimuth = qRadiansToDegrees(qAtan2(x, y));
    if (azimuth < 0.0) {
      azimuth += 360.0;
    }

    double deltaAngle = normalizeAngleDegrees(azimuth - prevAngle);
    if (qAbs(deltaAngle) > 150.0 && distance < 2.0) {
      deltaAngle = 0.0;
    }

    if (qAbs(deltaAngle) > 1e-3) {
      QJsonObject rotateCmd;
      rotateCmd["cmd"] = "rotate";
      QJsonObject rotateData;
      rotateData["delta_angle"] = deltaAngle;
      rotateCmd["data"] = rotateData;
      commands.append(rotateCmd);

      prevAngle = normalizeAngleDegrees(prevAngle + deltaAngle);
    } else {
      prevAngle = azimuth;
    }

    QJsonObject moveCmd;
    moveCmd["cmd"] = "move";
    QJsonObject moveData;
    moveData["distance_m"] = distance;
    moveCmd["data"] = moveData;
    commands.append(moveCmd);

    prevPos = nextPos;
  }

  return QJsonDocument(commands);
}
