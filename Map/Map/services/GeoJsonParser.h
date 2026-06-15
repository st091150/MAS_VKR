#pragma once

#include <QGeoView/QGVGlobal.h>
#include <QByteArray>
#include <QString>
#include <QVector>

struct GeoJsonParseResult {
  QVector<QGV::GeoPos> points;
  QString error;

  bool ok() const { return error.isEmpty(); }
};

namespace GeoJsonParser {
GeoJsonParseResult parseContour(const QByteArray& data);
}
