#include "GeoJsonParser.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtMath>

namespace {
QVector<QGV::GeoPos> parsePolygonOrLineString(const QJsonObject& geometry) {
  const QString type = geometry.value("type").toString();
  const QJsonArray coordsArray = geometry.value("coordinates").toArray();
  QVector<QGV::GeoPos> points;

  if (type == "Polygon" && !coordsArray.isEmpty()) {
    const QJsonArray ring = coordsArray[0].toArray();
    for (const auto& pt : ring) {
      const QJsonArray pair = pt.toArray();
      if (pair.size() >= 2) {
        points.push_back({pair[1].toDouble(), pair[0].toDouble()});
      }
    }
  } else if (type == "LineString") {
    for (const auto& pt : coordsArray) {
      const QJsonArray pair = pt.toArray();
      if (pair.size() >= 2) {
        points.push_back({pair[1].toDouble(), pair[0].toDouble()});
      }
    }
  }

  if (points.isEmpty()) {
    return points;
  }

  const auto& first = points.first();
  const auto& last = points.last();
  constexpr double eps = 1e-7;
  if (qAbs(first.latitude() - last.latitude()) > eps ||
      qAbs(first.longitude() - last.longitude()) > eps) {
    points.push_back(first);
  }

  return points;
}
}  // namespace

GeoJsonParseResult GeoJsonParser::parseContour(const QByteArray& data) {
  QJsonParseError parseError;
  const QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
  if (parseError.error != QJsonParseError::NoError) {
    return {{}, QObject::tr("Ошибка парсинга JSON: %1").arg(parseError.errorString())};
  }

  if (!doc.isObject()) {
    return {{}, QObject::tr("Некорректный формат GeoJSON")};
  }

  const QJsonObject root = doc.object();
  if (!root.contains("features") || !root.value("features").isArray()) {
    return {{}, QObject::tr("GeoJSON не содержит features")};
  }

  const QJsonArray features = root.value("features").toArray();
  for (const auto& featureValue : features) {
    if (!featureValue.isObject()) {
      continue;
    }

    const QJsonObject feature = featureValue.toObject();
    if (!feature.contains("geometry") || !feature.value("geometry").isObject()) {
      continue;
    }

    const QJsonObject geometry = feature.value("geometry").toObject();
    if (!geometry.contains("coordinates") || !geometry.value("coordinates").isArray()) {
      continue;
    }

    QVector<QGV::GeoPos> points = parsePolygonOrLineString(geometry);
    if (!points.isEmpty()) {
      return {points, {}};
    }
  }

  return {{}, QObject::tr("В файле не найден подходящий Polygon/LineString")};
}
