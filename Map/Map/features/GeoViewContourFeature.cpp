#include "GeoViewContourFeature.h"

#include "../dialogs/ContourPreviewDialog.h"
#include "../GeoViewWidget.h"
#include "../geometry/Contour.h"
#include "../geometry/GeoPolyline.h"
#include "../services/GeoJsonParser.h"
#include "../utils/MapLog.h"

#include <QCursor>
#include <QFile>
#include <QFileDialog>
#include <QGeoView/QGVCamera.h>
#include <QLineF>
#include <QMessageBox>
#include <QToolTip>
#include <algorithm>
#include <limits>

namespace {

bool openSegmentCrossesPriorEdges(GeoViewWidget& view, const QVector<QGV::GeoPos>& pts,
                                  const QGV::GeoPos& next) {
  const int n = static_cast<int>(pts.size());
  if (n < 2)
    return false;
  const QGV::GeoPos& last = pts[n - 1];
  for (int i = 0; i <= n - 3; ++i) {
    if (GeoViewContourFeature::segmentIntersection(view, pts[i], pts[i + 1], last, next))
      return true;
  }
  return false;
}

}  // namespace

void GeoViewContourFeature::zoomMapToContour(GeoViewWidget& view,
                                             const QVector<QGV::GeoPos>& points) {
  if (!view.mMap || points.isEmpty())
    return;
  double minLat = std::numeric_limits<double>::max();
  double maxLat = std::numeric_limits<double>::lowest();
  double minLon = std::numeric_limits<double>::max();
  double maxLon = std::numeric_limits<double>::lowest();
  for (const auto& p : points) {
    minLat = std::min(minLat, p.latitude());
    maxLat = std::max(maxLat, p.latitude());
    minLon = std::min(minLon, p.longitude());
    maxLon = std::max(maxLon, p.longitude());
  }
  const QGV::GeoRect bounds({minLat, minLon}, {maxLat, maxLon});
  view.mMap->cameraTo(QGVCameraActions(view.mMap).scaleTo(bounds));
}

bool GeoViewContourFeature::applyContourPolygon(GeoViewWidget& view,
                                                const QVector<QGV::GeoPos>& points) {
  if (points.size() < 3 || !view.mMap)
    return false;

  if (view.mContour)
    view.mContour->clear();
  view.clearCutouts();
  view.clearRouteLayer();

  view.mContour = new Contour(view.mMap);
  view.mContour->setPoints(points);
  view.mContour->draw();
  zoomMapToContour(view, points);
  view.updateInfoList();
  return true;
}

std::optional<QGV::GeoPos> GeoViewContourFeature::segmentIntersection(
    GeoViewWidget& view, const QGV::GeoPos& a, const QGV::GeoPos& b,
    const QGV::GeoPos& c, const QGV::GeoPos& d) {
  Q_UNUSED(view);
  const double x1 = a.longitude(), y1 = a.latitude();
  const double x2 = b.longitude(), y2 = b.latitude();
  const double x3 = c.longitude(), y3 = c.latitude();
  const double x4 = d.longitude(), y4 = d.latitude();

  const double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if (denom == 0.0)
    return std::nullopt;

  const double px =
      ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) /
      denom;
  const double py =
      ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) /
      denom;

  auto onSegment = [](double p, double q1, double q2) {
    return (p >= std::min(q1, q2) - 1e-9) && (p <= std::max(q1, q2) + 1e-9);
  };

  if (onSegment(px, x1, x2) && onSegment(px, x3, x4) && onSegment(py, y1, y2) &&
      onSegment(py, y3, y4)) {
    return QGV::GeoPos(py, px);
  }
  return std::nullopt;
}

QVector<QGV::GeoPos> GeoViewContourFeature::polygonSelfIntersections(
    GeoViewWidget& view, const QVector<QGV::GeoPos>& points) {
  QVector<QGV::GeoPos> intersections;
  const size_t n = points.size();
  if (n < 4)
    return intersections;

  for (size_t i = 0; i < n - 1; ++i) {
    for (size_t j = i + 1; j < n - 1; ++j) {
      if (j == i + 1)
        continue;
      if (i == 0 && j == n - 2)
        continue;
      auto pt = segmentIntersection(view, points[i], points[i + 1], points[j],
                                    points[j + 1]);
      if (pt)
        intersections.push_back(*pt);
    }
  }
  return intersections;
}

void GeoViewContourFeature::addContour(GeoViewWidget& view) {
  cancelDrawContour(view);
  const QString fileName = QFileDialog::getOpenFileName(
      &view, view.tr("Выберите файл GeoJSON"), QString(), view.tr("GeoJSON (*.geojson)"));
  if (fileName.isEmpty())
    return;

  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(logUi) << view.tr("Не удалось открыть файл");
    return;
  }
  const QByteArray data = file.readAll();
  file.close();

  const GeoJsonParseResult result = GeoJsonParser::parseContour(data);
  if (!result.ok()) {
    qCWarning(logUi) << result.error;
    return;
  }

  const QVector<QGV::GeoPos> points = result.points;
  const QVector<QGV::GeoPos> intersections = polygonSelfIntersections(view, points);

  if (!intersections.empty()) {
    view.mPrevDialog = new ContourPreviewDialog(points, intersections, &view);
    view.mPrevDialog->exec();
    return;
  }

  applyContourPolygon(view, points);
  view.setUiStatus(view.tr("Контур загружен из GeoJSON"), GeoViewWidget::UiStatusLevel::Success);
}

void GeoViewContourFeature::startDrawContourMode(GeoViewWidget& view) {
  view.mActiveCutoutPolygon.clear();
  view.mActiveContourPolygon.clear();
  view.setInteractionMode(GeoViewWidget::InteractionMode::DrawContour);
  redrawActiveContourDraft(view);
  const QString hint =
      view.mContour && view.mContour->points().size() >= 3
          ? QObject::tr("Режим контура: ставьте точки на карте. Клик по первой точке "
                        "завершит полигон (текущий контур будет заменён).")
          : QObject::tr("Режим контура: ставьте точки на карте. Клик по первой точке "
                        "завершит полигон.");
  view.setUiStatus(hint, GeoViewWidget::UiStatusLevel::Info);
}

void GeoViewContourFeature::handleDrawContourClick(GeoViewWidget& view, const QGV::GeoPos& pos) {
  if (view.mInteractionMode != GeoViewWidget::InteractionMode::DrawContour)
    return;
  if (!view.mMap || !view.mMap->getProjection())
    return;

  if (view.mActiveContourPolygon.size() >= 3) {
    const QPointF first = view.mMap->getProjection()->geoToProj(view.mActiveContourPolygon.first());
    const QPointF current = view.mMap->getProjection()->geoToProj(pos);
    if (QLineF(first, current).length() <= 1.5) {
      QVector<QGV::GeoPos> closed = view.mActiveContourPolygon;
      if (closed.front() != closed.back())
        closed.push_back(closed.front());
      const QVector<QGV::GeoPos> intersections = polygonSelfIntersections(view, closed);
      if (!intersections.isEmpty()) {
        view.mPrevDialog = new ContourPreviewDialog(closed, intersections, &view);
        view.mPrevDialog->exec();
        view.mActiveContourPolygon.clear();
        redrawActiveContourDraft(view);
        return;
      }
      if (applyContourPolygon(view, closed)) {
        view.mActiveContourPolygon.clear();
        view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
        redrawActiveContourDraft(view);
        view.setUiStatus(QObject::tr("Контур задан вручную (%1 вершин)").arg(closed.size() - 1),
                         GeoViewWidget::UiStatusLevel::Success);
      }
      return;
    }
  }

  if (!view.mActiveContourPolygon.isEmpty() &&
      openSegmentCrossesPriorEdges(view, view.mActiveContourPolygon, pos)) {
    const QString msg = QObject::tr("Контур не должен самопересекаться.");
    view.setUiStatus(msg, GeoViewWidget::UiStatusLevel::Warning);
    QToolTip::showText(QCursor::pos(), msg, &view);
    return;
  }

  view.mActiveContourPolygon.push_back(pos);
  redrawActiveContourDraft(view);
  view.setUiStatus(QObject::tr("Вершин контура: %1. Для завершения кликните по первой точке.")
                       .arg(view.mActiveContourPolygon.size()),
                   GeoViewWidget::UiStatusLevel::Info);
}

void GeoViewContourFeature::cancelDrawContour(GeoViewWidget& view) {
  view.mActiveContourPolygon.clear();
  if (view.mInteractionMode == GeoViewWidget::InteractionMode::DrawContour) {
    view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
  }
  redrawActiveContourDraft(view);
}

void GeoViewContourFeature::redrawActiveContourDraft(GeoViewWidget& view) {
  if (!view.mCutoutDraftLayer)
    return;
  view.mCutoutDraftLayer->deleteItems();
  if (view.mActiveContourPolygon.size() < 2)
    return;
  QPen activePen(QColor(220, 60, 60, 220), 2.5);
  activePen.setStyle(Qt::DotLine);
  auto* poly = new GeoPolyline(view.mMap);
  poly->setPen(activePen);
  poly->points = view.mActiveContourPolygon;
  view.mCutoutDraftLayer->addItem(poly);
}

void GeoViewContourFeature::removeContour(GeoViewWidget& view) {
  cancelDrawContour(view);
  if (!view.mContour)
    return;
  view.mContour->clear();
  view.clearCutouts();
  view.clearRouteLayer();
  view.updateInfoList();
  view.setUiStatus(view.tr("Контур удалён"), GeoViewWidget::UiStatusLevel::Info);
}
