#include "GeoViewWidget.h"
#include "features/GeoViewCommandFeature.h"
#include "features/GeoViewContourFeature.h"
#include "features/GeoViewCutoutFeature.h"
#include "features/GeoViewInteractionFeature.h"
#include "features/GeoViewRouteFeature.h"
#include "features/GeoViewRobotFeature.h"
#include "geometry/GeoPolyline.h"
#include "ui/GeoViewUiComposer.h"
#include "utils/MapLog.h"
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSizePolicy>
#include <QStackedWidget>
#include <QFrame>
#include <QTimer>
#include <QVBoxLayout>


#include <helpers.h>
#include <qlabel.h>
#include <rectangle.h>

#include <QGeoView/QGVDrawItem.h>
#include <QGeoView/QGVLayerOSM.h>
#include <QGeoView/QGVWidgetCompass.h>
#include <QGeoView/QGVWidgetScale.h>
#include <QGeoView/QGVWidgetZoom.h>
#include <QGeoView/Raster/QGVIcon.h>
#include <QGeoView/Raster/QGVImage.h>
#include <QColor>
#include <algorithm>

namespace {
}  // namespace

GeoViewWidget::GeoViewWidget(QWidget* parent) : QWidget(parent) {
  setWindowTitle(tr("Map"));
  Helpers::setupCachedNetworkAccessManager(this);

  auto* layout = new QVBoxLayout(this);
  initMap();
  initLayers();
  initUi(layout);
  scheduleInitialCamera();
  preloadImages();
  connectSignals();
}

GeoViewWidget::~GeoViewWidget() {
  clearAll();
}

QGV::GeoRect GeoViewWidget::targetArea() const {
  return QGV::GeoRect(QGV::GeoPos(50, 14), QGV::GeoPos(52, 15));
}

void GeoViewWidget::initMap() {
  mMap = new QGVMap(this);
  mMap->setMouseTracking(true);

  mMap->addWidget(new QGVWidgetCompass());
  mMap->addWidget(new QGVWidgetZoom());
  mMap->addWidget(new QGVWidgetScale());

  // CyclOSM (OpenStreetMap France): иной стиль, чем стандартный OSM Carto;
  // подписи берутся из OSM (для РФ часто кириллица).
  auto* osmLayer = new QGVLayerOSM(QStringLiteral(
      "https://a.tile-cyclosm.openstreetmap.fr/cyclosm/${z}/${x}/${y}.png"));
  osmLayer->setName(QStringLiteral("Карта CyclOSM"));
  osmLayer->setDescription(
      QStringLiteral("© участники OpenStreetMap; стиль CyclOSM © OpenStreetMap France"));
  mMap->addItem(osmLayer);
}

void GeoViewWidget::initLayers() {
  mLayers = GeoViewMapLayers::createAndAttach(*mMap);

  // Backward-compatible aliases.
  mLayer = mLayers.main;
  mRouteLayer = mLayers.route;
  mCutoutLayer = mLayers.cutout;
  mCutoutDraftLayer = mLayers.cutoutDraft;
  mRobotLayer = mLayers.robot;
}

void GeoViewWidget::initUi(QVBoxLayout* rootLayout) {
  rootLayout->setContentsMargins(6, 6, 6, 6);
  rootLayout->setSpacing(6);

  auto* contentContainer = new QWidget(this);
  auto* contentLayout = new QHBoxLayout(contentContainer);
  contentLayout->setContentsMargins(0, 0, 0, 0);
  contentLayout->setSpacing(10);
  contentLayout->addWidget(mMap, 1);

  auto* sidePanel = new QFrame(contentContainer);
  sidePanel->setObjectName("sidePanel");
  sidePanel->setMinimumWidth(360);
  sidePanel->setMaximumWidth(560);
  auto* sideLayout = new QVBoxLayout(sidePanel);
  sideLayout->setContentsMargins(8, 8, 8, 8);
  sideLayout->setSpacing(8);

  auto* mainPage = new QWidget(sidePanel);
  auto* mainLay = new QVBoxLayout(mainPage);
  mainLay->setContentsMargins(0, 0, 0, 0);
  mainLay->setSpacing(8);

  QGroupBox* controlsCard = createOptionsList();
  controlsCard->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

  auto* controlsScroll = new QScrollArea(mainPage);
  controlsScroll->setWidgetResizable(true);
  controlsScroll->setFrameShape(QFrame::NoFrame);
  controlsScroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  controlsScroll->setWidget(controlsCard);

  QGroupBox* infoCard = createInfoList();
  mainLay->addWidget(controlsScroll, 2);
  mainLay->addWidget(infoCard, 1);

  mParamsWidget = createRouteParamsWidget();
  mParamsWidget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

  auto* paramsPage = new QWidget(sidePanel);
  auto* paramsLay = new QVBoxLayout(paramsPage);
  paramsLay->setContentsMargins(0, 0, 0, 0);
  paramsLay->setSpacing(8);
  auto* paramsBackBtn = new QPushButton(tr("Назад к панели управления"));
  paramsBackBtn->setObjectName("paramsBackButton");
  paramsLay->addWidget(paramsBackBtn, 0);
  paramsLay->addWidget(mParamsWidget, 1);

  mSidePanelStack = new QStackedWidget(sidePanel);
  mSidePanelStack->setObjectName("sidePanelStack");
  mSidePanelStack->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  mSidePanelStack->addWidget(mainPage);
  mSidePanelStack->addWidget(paramsPage);
  sideLayout->addWidget(mSidePanelStack, 1);

  connect(paramsBackBtn, &QPushButton::clicked, this, [this]() {
    if (mSidePanelStack)
      mSidePanelStack->setCurrentIndex(0);
  });

  contentLayout->addWidget(sidePanel, 0);
  rootLayout->addWidget(contentContainer, 1);

  sidePanel->setStyleSheet(
      "#sidePanel { background: qlineargradient(x1:0, y1:0, x2:0, y2:1, "
      "stop:0 #111827, stop:1 #0B1120); border: 1px solid #263244; border-radius: 16px; }"
      "#sidePanelStack { background: transparent; border: none; }"
      "#paramsBackButton { min-height: 36px; padding: 6px 14px; color: #F8FAFC; "
      "background-color: #334155; border: 1px solid #475569; border-radius: 9px; font-weight: 700; }"
      "#paramsBackButton:hover { background-color: #475569; }");
  setUiStatus(tr("Готово к работе"), UiStatusLevel::Info);
}

void GeoViewWidget::connectSignals() {
  connect(mMap, &QGVMap::mapMousePress, this, &GeoViewWidget::onMapMousePress);
}

void GeoViewWidget::scheduleInitialCamera() {
  QTimer::singleShot(100, this, [this]() {
    mMap->cameraTo(QGVCameraActions(mMap).scaleTo(targetArea()));
  });
}

void GeoViewWidget::onMapMousePress(QPointF projPos) {
  GeoViewInteractionFeature::onMapMousePress(*this, projPos);
}

QGroupBox* GeoViewWidget::createOptionsList() {
  return GeoViewUiComposer::createOptionsList(*this);
}

QGroupBox* GeoViewWidget::createInfoList() {
  return GeoViewUiComposer::createInfoList(*this);
}

QWidget* GeoViewWidget::createRouteParamsWidget() {
  return GeoViewUiComposer::createRouteParamsWidget(*this);
}

void GeoViewWidget::preloadImages() {
  //loadImage(mRobotIcon, QUrl{ "https://earth.google.com/images/kml-icons/track-directional/track-0.png" });
  mRobotIcon.load(":/icons/robot_icons/robot_icon.png");
}

void GeoViewWidget::loadImage(QImage& dest, QUrl url) {
  QNetworkRequest request(url);
  request.setRawHeader("User-Agent",
                       "Mozilla/5.0 (Windows; U; MSIE "
                       "6.0; Windows NT 5.1; SV1; .NET "
                       "CLR 2.0.50727)");
  request.setAttribute(QNetworkRequest::HttpPipeliningAllowedAttribute, true);
  request.setAttribute(QNetworkRequest::CacheLoadControlAttribute,
                       QNetworkRequest::PreferCache);

  QNetworkReply* reply = QGV::getNetworkManager()->get(request);
  connect(reply, &QNetworkReply::finished, reply, [reply, &dest]() {
    if (reply->error() != QNetworkReply::NoError) {
      qgvCritical() << "ERROR" << reply->errorString();
      reply->deleteLater();
      return;
    }
    dest.loadFromData(reply->readAll());
    reply->deleteLater();
  });

  qgvDebug() << "request" << url;
}

std::optional<QGV::GeoPos> GeoViewWidget::segmentIntersection(
    const QGV::GeoPos& a, const QGV::GeoPos& b, const QGV::GeoPos& c,
    const QGV::GeoPos& d) {
  // Contour algorithms are delegated to the dedicated feature module.
  return GeoViewContourFeature::segmentIntersection(*this, a, b, c, d);
}

QVector<QGV::GeoPos> GeoViewWidget::polygonSelfIntersections(
    const QVector<QGV::GeoPos>& points) {
  return GeoViewContourFeature::polygonSelfIntersections(*this, points);
}

void GeoViewWidget::addContour() {
  GeoViewContourFeature::addContour(*this);
}

QPointF GeoViewWidget::moveAlongContour(const ClipperLib::Path& contour,
                                        const QPointF& startPt,
                                        double stepMeters, bool forward) {
  return GeoViewRouteFeature::moveAlongContour(*this, contour, startPt, stepMeters, forward);
}

void GeoViewWidget::buildRouteWithAngleForCustomRoute(
    double stepMeters, double angleDegrees, double contourOffset,
    const QGV::GeoPos& startPointParam, const QGV::GeoPos& endPointParam,
    bool rightSide, bool debugMode, RouteAlgo::PipelineStage visualizationStage) {
  mRouteVisualizationStage = visualizationStage;
  GeoViewRouteFeature::buildRouteWithAngleForCustomRoute(
      *this, stepMeters, angleDegrees, contourOffset, startPointParam, endPointParam, rightSide,
      debugMode, visualizationStage);
}

void GeoViewWidget::setRouteVisualizationStage(RouteAlgo::PipelineStage stage, int stepLimit) {
  mRouteVisualizationStage = stage;
  GeoViewRouteFeature::setVisualizationStage(*this, stage, stepLimit);
}

void GeoViewWidget::startCutoutPolygonMode() {
  GeoViewCutoutFeature::startCutoutPolygonMode(*this);
}

void GeoViewWidget::startDrawContourOnMap() {
  GeoViewContourFeature::startDrawContourMode(*this);
}

void GeoViewWidget::handleDrawContourClick(const QGV::GeoPos& pos) {
  GeoViewContourFeature::handleDrawContourClick(*this, pos);
}

void GeoViewWidget::cancelDrawContour() {
  GeoViewContourFeature::cancelDrawContour(*this);
}

void GeoViewWidget::handleCutoutPolygonClick(const QGV::GeoPos& pos) {
  GeoViewCutoutFeature::handleCutoutPolygonClick(*this, pos);
}

bool GeoViewWidget::finishCutoutPolygon() {
  return GeoViewCutoutFeature::finishCutoutPolygon(*this);
}

void GeoViewWidget::undoLastCutout() {
  GeoViewCutoutFeature::undoLastCutout(*this);
}

void GeoViewWidget::redrawCutouts() {
  GeoViewCutoutFeature::redrawCutouts(*this);
}

void GeoViewWidget::redrawActiveCutout() {
  GeoViewCutoutFeature::redrawActiveCutout(*this);
}

void GeoViewWidget::clearCutouts() {
  GeoViewCutoutFeature::clearCutouts(*this);
}

QVector<QGV::GeoPos> GeoViewWidget::buildRouteWithAngle(
    double stepMeters, double angleDegrees, double offsetFromContour,
    double offsetCut  // не нужен, смотреть на buildRouteWithAngleForCustomRoute
) {
  return GeoViewRouteFeature::buildRouteWithAngle(
      *this, stepMeters, angleDegrees, offsetFromContour, offsetCut);
}

void GeoViewWidget::removeContour() {
  GeoViewContourFeature::removeContour(*this);
}

void GeoViewWidget::updateInfoList() {
  if (!mInfoWidget)
    return;

  mInfoWidget->clear();

  for (int i = 0; i < mLayer->countItems(); i++) {
    QGVItem* item = mLayer->getItem(i);
    mInfoWidget->addItem(item->metaObject()->className());
  }
}

void GeoViewWidget::setInteractionMode(InteractionMode mode) {
  mInteractionMode = mode;
  refreshContourDrawingUi();
}

void GeoViewWidget::refreshContourDrawingUi() {
  if (!mCancelDrawContourButton)
    return;
  mCancelDrawContourButton->setVisible(mInteractionMode == InteractionMode::DrawContour);
}

GeoViewWidget::InteractionMode GeoViewWidget::interactionMode() const {
  return mInteractionMode;
}

void GeoViewWidget::setUiStatus(const QString& text, UiStatusLevel level) {
  if (!mStatusLabel) {
    return;
  }
  QColor color(70, 70, 70);
  switch (level) {
    case UiStatusLevel::Success:
      color = QColor(20, 120, 60);
      break;
    case UiStatusLevel::Warning:
      color = QColor(160, 110, 20);
      break;
    case UiStatusLevel::Error:
      color = QColor(170, 40, 40);
      break;
    case UiStatusLevel::Info:
    default:
      break;
  }
  mStatusLabel->setText(text);
  mStatusLabel->setStyleSheet(QStringLiteral("QLabel { font-weight: 600; color: %1; }").arg(color.name()));
}

void GeoViewWidget::handleMapClickStartRoutePoint(const QGV::GeoPos& pos) {
  GeoViewRouteFeature::handleMapClickStartRoutePoint(*this, pos);
}

void GeoViewWidget::handleMapClick(const QGV::GeoPos& pos) {
  GeoViewRouteFeature::handleMapClick(*this, pos);
}

void GeoViewWidget::drawRoute(QGVLayer* layer, const QVector<QGV::GeoPos>& pts,
                              const QPen& pen, bool drawArrow,
                              bool replaceExisting) {
  GeoViewRouteFeature::drawRoute(*this, layer, pts, pen, drawArrow, replaceExisting);
}

void GeoViewWidget::toggleManualRouteMode() {
  GeoViewRouteFeature::toggleManualRouteMode(*this);
}

void GeoViewWidget::addRobot(double latitude, double longitude, double angle) {
  GeoViewRobotFeature::addRobot(*this, latitude, longitude, angle);
}

void GeoViewWidget::updateRobot(double latitude, double longitude,
                                double angle) {
  GeoViewRobotFeature::updateRobot(*this, latitude, longitude, angle);
}

void GeoViewWidget::createRoute() {
  GeoViewCommandFeature::createRoute(*this);
}

void GeoViewWidget::showRobotCommandsJson() {
  GeoViewCommandFeature::showRobotCommandsJson(*this);
}

QJsonDocument GeoViewWidget::getRouteCommands() const {
  return GeoViewCommandFeature::getRouteCommands(*this);
}

double GeoViewWidget::haversineDistance(const QGV::GeoPos& start,
                                        const QGV::GeoPos& end) {
  double lat1 = qDegreesToRadians(start.latitude());
  double lat2 = qDegreesToRadians(end.latitude());
  double dLat = lat2 - lat1;
  double dLon = qDegreesToRadians(end.longitude() - start.longitude());

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return EARTH_RADIUS_METERS * c;
}

double GeoViewWidget::calculateBearing(const QGV::GeoPos& start,
                                       const QGV::GeoPos& end) {
  double lat1 = qDegreesToRadians(start.latitude());
  double lat2 = qDegreesToRadians(end.latitude());
  double dLon = qDegreesToRadians(end.longitude() - start.longitude());

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  double bearingRad = atan2(y, x);
  double bearingDeg = qRadiansToDegrees(bearingRad);

  if (bearingDeg > 180)
    bearingDeg -= 360;
  if (bearingDeg < -180)
    bearingDeg += 360;

  return bearingDeg;
}

QPointF GeoViewWidget::computeGazeboPoint(const QGV::GeoPos& start,
                                          const QGV::GeoPos& end) {
  double distance = haversineDistance(start, end);
  double bearingDeg = calculateBearing(start, end);
  double bearingRad = qDegreesToRadians(bearingDeg);

  double xEast = distance * sin(bearingRad);   // Восток
  double yNorth = distance * cos(bearingRad);  // Север

  return QPointF(xEast, yNorth);  // Локальные координаты ENU
}

double GeoViewWidget::calculateRosYaw(const QGV::GeoPos& start,
                                      const QGV::GeoPos& end) {
  double bearingDeg = calculateBearing(start, end);
  double bearingRad = qDegreesToRadians(bearingDeg);

  double yawRos = M_PI_2 - bearingRad;

  if (yawRos > M_PI)
    yawRos -= 2 * M_PI;
  if (yawRos < -M_PI)
    yawRos += 2 * M_PI;

  return yawRos;
}

QJsonDocument GeoViewWidget::generateGazeboJson() {
  return GeoViewCommandFeature::generateGazeboJson(*this);
}

QJsonDocument GeoViewWidget::generateGazeboWaypointsJson() {
  return GeoViewCommandFeature::generateGazeboWaypointsJson(*this);
}

void GeoViewWidget::clearRouteLayer() {
  if (mRouteLayer) mRouteLayer->deleteItems();
  GeoViewRouteFeature::invalidateRouteAnchorMarkers(*this);

  if (!mState.routePoints.empty())
    mState.routePoints.clear();

  mState.routeCommands.reset();
}

void GeoViewWidget::clearRobotLayer() {
  if (mRobotLayer)
    mRobotLayer->deleteItems();

  if (mRobotItem.item) {
    mRobotItem.item = nullptr;
    mRobotItem.pos = QGV::GeoPos(0, 0);
    mRobotItem.angle = 0.;
  }
}

void GeoViewWidget::clearRouteCommands() {
  mState.routeCommands.reset();
}

void GeoViewWidget::clearAll() {
  cancelDrawContour();
  clearMainLayer();
  clearRouteLayer();
  clearCutouts();
  clearRobotLayer();
  clearRouteCommands();

  if (mContour)
    mContour->clear();

  if (mInfoWidget)
    mInfoWidget->clear();

  if (mPrevDialog) {
    delete mPrevDialog;
    mPrevDialog = nullptr;
  };

  updateInfoList();
  setInteractionMode(InteractionMode::Idle);
  setUiStatus(tr("Все данные очищены"), UiStatusLevel::Info);

  qCDebug(logUi) << "Все данные очищены.";
}
