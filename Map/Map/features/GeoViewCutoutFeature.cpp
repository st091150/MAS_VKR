#include "GeoViewCutoutFeature.h"
 
#include "../GeoViewWidget.h"
#include "../geometry/GeoPolyline.h"
#include "../utils/ClipperUtils.h"
 
#include "GeoViewContourFeature.h"

#include <QCursor>
#include <QLineF>
#include <QMessageBox>
#include <QToolTip>
#include <QObject>
 
namespace {
ClipperLib::Paths buildCutoutRegion(QGVMap* map, const Contour* contour,
                                    const QVector<QVector<QGV::GeoPos>>& cutoutPolygons) {
  if (!map || !map->getProjection() || !contour || contour->points().size() < 3) {
    return {};
  }

  QVector<QPointF> contourProj;
  contourProj.reserve(contour->points().size());
  for (const auto& p : contour->points()) {
    contourProj.push_back(map->getProjection()->geoToProj(p));
  }

  ClipperLib::Paths region{ClipperUtils::toClipPath(contourProj)};
  ClipperUtils::orientClipperPathOuter(region.back());
  ClipperLib::Paths cutouts;
  cutouts.reserve(cutoutPolygons.size());
  for (const auto& cutout : cutoutPolygons) {
    if (cutout.size() < 3)
      continue;
    QVector<QPointF> cutoutProj;
    cutoutProj.reserve(cutout.size());
    for (const auto& p : cutout) {
      cutoutProj.push_back(map->getProjection()->geoToProj(p));
    }
    ClipperLib::Path cut = ClipperUtils::toClipPath(cutoutProj);
    ClipperUtils::orientClipperPathOuter(cut);
    cutouts.push_back(std::move(cut));
  }
  if (cutouts.empty())
    return ClipperUtils::unionPaths(region);
  return ClipperUtils::difference(ClipperUtils::unionPaths(region),
                                  ClipperUtils::unionPaths(cutouts));
}

QVector<QVector<QGV::GeoPos>> outerPreviewFromRegion(QGVMap* map,
                                                     const ClipperLib::Paths& region) {
  QVector<QVector<QGV::GeoPos>> out;
  if (!map || !map->getProjection())
    return out;
  for (const auto& path : region) {
    if (path.size() < 3 || ClipperLib::Area(path) <= 0.0)
      continue;
    QVector<QGV::GeoPos> geo;
    geo.reserve(static_cast<int>(path.size()) + 1);
    for (const auto& p : path) {
      geo.push_back(map->getProjection()->projToGeo(ClipperUtils::fromClip(p)));
    }
    if (!geo.isEmpty() && geo.front() != geo.back())
      geo.push_back(geo.front());
    out.push_back(std::move(geo));
  }
  return out;
}

bool cutoutOpenSegmentCrossesPriorEdges(GeoViewWidget& view,
                                        const QVector<QGV::GeoPos>& pts,
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

void GeoViewCutoutFeature::startCutoutPolygonMode(GeoViewWidget& view) {
  GeoViewContourFeature::cancelDrawContour(view);
  if (!view.mContour || view.mContour->points().size() < 3) {
    view.setUiStatus(QObject::tr("Сначала добавьте рабочий контур"), GeoViewWidget::UiStatusLevel::Warning);
    QMessageBox::warning(&view, QObject::tr("Нет контура"),
                         QObject::tr("Сначала задайте рабочую область, затем рисуйте вырезы."));
    return;
  }
  view.mActiveCutoutPolygon.clear();
  view.setInteractionMode(GeoViewWidget::InteractionMode::CutoutPolygon);
  redrawActiveCutout(view);
  view.setUiStatus(QObject::tr("Режим выреза: ставьте точки, клик по первой точке завершит контур"),
                   GeoViewWidget::UiStatusLevel::Info);
}
 
void GeoViewCutoutFeature::handleCutoutPolygonClick(GeoViewWidget& view, const QGV::GeoPos& pos) {
  if (view.mInteractionMode != GeoViewWidget::InteractionMode::CutoutPolygon)
    return;
  if (!view.mMap || !view.mMap->getProjection())
    return;

  if (!view.mContour || view.mContour->points().size() < 3) {
    view.setUiStatus(QObject::tr("Сначала добавьте рабочий контур"), GeoViewWidget::UiStatusLevel::Warning);
    return;
  }

  if (view.mActiveCutoutPolygon.size() >= 3) {
    const QPointF first = view.mMap->getProjection()->geoToProj(view.mActiveCutoutPolygon.first());
    const QPointF current = view.mMap->getProjection()->geoToProj(pos);
    const double closeDistance = QLineF(first, current).length();
    if (closeDistance <= 1.5) {
      QVector<QGV::GeoPos> closed = view.mActiveCutoutPolygon;
      if (closed.front() != closed.back())
        closed.push_back(closed.front());
      if (!view.polygonSelfIntersections(closed).isEmpty()) {
        const QString msg = QObject::tr("Контур выреза не должен самопересекаться.");
        view.setUiStatus(msg, GeoViewWidget::UiStatusLevel::Warning);
        QToolTip::showText(QCursor::pos(), msg, &view);
        return;
      }
      finishCutoutPolygon(view);
      return;
    }
  }
 
  if (!view.mActiveCutoutPolygon.isEmpty() &&
      cutoutOpenSegmentCrossesPriorEdges(view, view.mActiveCutoutPolygon, pos)) {
    const QString msg = QObject::tr("Контур выреза не должен самопересекаться.");
    view.setUiStatus(msg, GeoViewWidget::UiStatusLevel::Warning);
    QToolTip::showText(QCursor::pos(), msg, &view);
    return;
  }
 
  view.mActiveCutoutPolygon.push_back(pos);
  redrawActiveCutout(view);
  view.setUiStatus(QObject::tr("Точек выреза: %1. Для завершения кликните по первой точке.")
                       .arg(view.mActiveCutoutPolygon.size()),
                   GeoViewWidget::UiStatusLevel::Info);
}
 
bool GeoViewCutoutFeature::finishCutoutPolygon(GeoViewWidget& view) {
  if (view.mActiveCutoutPolygon.size() < 3) {
    view.setUiStatus(QObject::tr("Для выреза нужно минимум 3 точки"), GeoViewWidget::UiStatusLevel::Warning);
    return false;
  }
 
  QVector<QGV::GeoPos> closed = view.mActiveCutoutPolygon;
  if (closed.front() != closed.back())
    closed.push_back(closed.front());
 
  const QVector<QGV::GeoPos> intersections = view.polygonSelfIntersections(closed);
  if (!intersections.empty()) {
    const QString msg = QObject::tr("Контур выреза не должен самопересекаться.");
    view.setUiStatus(msg, GeoViewWidget::UiStatusLevel::Warning);
    QToolTip::showText(QCursor::pos(), msg, &view);
    redrawActiveCutout(view);
    return false;
  }
 
  view.mCutoutPolygons.push_back(closed);
  view.mSplitRegionPreview = outerPreviewFromRegion(
      view.mMap, buildCutoutRegion(view.mMap, view.mContour, view.mCutoutPolygons));
  view.mActiveCutoutPolygon.clear();
  view.clearRouteLayer();
  redrawCutouts(view);
  view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
  if (view.mSplitRegionPreview.size() > 1) {
    view.setUiStatus(QObject::tr("Внимание: рабочая область стала мультиконтуром (%1 частей)")
                         .arg(view.mSplitRegionPreview.size()),
                     GeoViewWidget::UiStatusLevel::Warning);
  } else {
    view.setUiStatus(QObject::tr("Вырез добавлен: %1").arg(view.mCutoutPolygons.size()),
                     GeoViewWidget::UiStatusLevel::Success);
  }
  return true;
}
 
void GeoViewCutoutFeature::undoLastCutout(GeoViewWidget& view) {
  view.mActiveCutoutPolygon.clear();
  if (view.mCutoutPolygons.empty()) {
    redrawCutouts(view);
    view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
    view.setUiStatus(QObject::tr("Нет вырезов для удаления"), GeoViewWidget::UiStatusLevel::Info);
    return;
  }
  view.mCutoutPolygons.pop_back();
  view.mSplitRegionPreview = outerPreviewFromRegion(
      view.mMap, buildCutoutRegion(view.mMap, view.mContour, view.mCutoutPolygons));
  if (view.mSplitRegionPreview.size() <= 1)
    view.mSplitRegionPreview.clear();
  view.clearRouteLayer();
  redrawCutouts(view);
  view.setInteractionMode(GeoViewWidget::InteractionMode::Idle);
  view.setUiStatus(QObject::tr("Последний вырез удален. Осталось: %1").arg(view.mCutoutPolygons.size()),
                   GeoViewWidget::UiStatusLevel::Info);
}
 
void GeoViewCutoutFeature::redrawCutouts(GeoViewWidget& view) {
  if (!view.mCutoutLayer)
    return;
  view.mCutoutLayer->deleteItems();
  if (view.mCutoutDraftLayer)
    view.mCutoutDraftLayer->deleteItems();
 
  auto addPolygon = [&view](QGVLayer* layer, const QVector<QGV::GeoPos>& pts, const QPen& pen) {
    if (pts.size() < 2)
      return;
    auto* poly = new GeoPolyline(view.mMap);
    poly->setPen(pen);
    poly->points = pts;
    layer->addItem(poly);
  };
 
  QPen savedPen(QColor(255, 150, 70, 220), 2.0);
  savedPen.setStyle(Qt::DashLine);
  for (const auto& polygon : view.mCutoutPolygons) {
    addPolygon(view.mCutoutLayer, polygon, savedPen);
  }
 
  redrawActiveCutout(view);
}
 
void GeoViewCutoutFeature::redrawActiveCutout(GeoViewWidget& view) {
  if (!view.mCutoutDraftLayer)
    return;
  view.mCutoutDraftLayer->deleteItems();

  if (view.mInteractionMode == GeoViewWidget::InteractionMode::DrawContour &&
      view.mActiveContourPolygon.size() >= 2) {
    GeoViewContourFeature::redrawActiveContourDraft(view);
    return;
  }
 
  if (view.mActiveCutoutPolygon.isEmpty() && view.mSplitRegionPreview.size() > 1) {
    auto multiContourPreviewColor = [](int partIndex) {
      const int hue = (partIndex * 53 + 17) % 360;
      return QColor::fromHsv(hue, 255, 255, 255);
    };
    for (int i = 0; i < view.mSplitRegionPreview.size(); ++i) {
      QPen splitPen(multiContourPreviewColor(i), 4.0);
      splitPen.setStyle(Qt::SolidLine);
      auto* poly = new GeoPolyline(view.mMap);
      poly->setPen(splitPen);
      poly->points = view.mSplitRegionPreview[static_cast<size_t>(i)];
      view.mCutoutDraftLayer->addItem(poly);
    }
    return;
  }
 
  if (view.mActiveCutoutPolygon.size() >= 2) {
    QPen activePen(QColor(255, 210, 80, 230), 2.0);
    activePen.setStyle(Qt::DotLine);
    auto* poly = new GeoPolyline(view.mMap);
    poly->setPen(activePen);
    poly->points = view.mActiveCutoutPolygon;
    view.mCutoutDraftLayer->addItem(poly);
  }
}
 
void GeoViewCutoutFeature::clearCutouts(GeoViewWidget& view) {
  view.mCutoutPolygons.clear();
  view.mActiveCutoutPolygon.clear();
  view.mSplitRegionPreview.clear();
  if (view.mCutoutLayer)
    view.mCutoutLayer->deleteItems();
  if (view.mCutoutDraftLayer)
    view.mCutoutDraftLayer->deleteItems();
}
