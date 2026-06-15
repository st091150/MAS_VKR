#pragma once

#include <QGeoView/QGVGlobal.h>
#include <QJsonDocument>
#include <QVector>

namespace RouteCommandBuilder {
QJsonDocument buildRouteCommands(const QGV::GeoPos& startPos, double startAngleDegrees,
                                 const QVector<QGV::GeoPos>& routePoints);
}
