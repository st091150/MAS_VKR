#pragma once
#include "dialogs/ContourPreviewDialog.h"
#include "features/algorithm/pipeline/RoutePipelineTypes.h"
#include "geometry/Contour.h"
#include "state/GeoViewState.h"

#include "GeoViewMapLayers.h"

#include "clipper/clipper.hpp"

#include <QGroupBox>
#include <QImage>
#include <QListWidget>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QWidget>
#include <QObject>


#include <QGeoView/QGVLayer.h>
#include <QGeoView/QGVMap.h>
#include <QGeoView/Raster/QGVIcon.h>
#include <qjsondocument.h>
#include <optional>


constexpr double EARTH_RADIUS_METERS = 6371000.0;

class GeoViewUiComposer;
class GeoViewContourFeature;
class GeoViewRouteFeature;
class GeoViewRobotFeature;
class GeoViewCommandFeature;
class GeoViewCutoutFeature;
class GeoViewInteractionFeature;
class QVBoxLayout;

class GeoViewWidget : public QWidget {
  Q_OBJECT

 private:
  enum class InteractionMode {
    Idle,
    ManualRoute,
    SelectStartPoint,
    SelectEndPoint,
    CutoutPolygon,
    /// Interactive polygon for the field boundary (contour).
    DrawContour,
    /// Next map click sets / moves the robot icon at that geo position.
    PlaceRobot
  };

  enum class UiStatusLevel {
    Info,
    Success,
    Warning,
    Error
  };

  struct RobotItem {
    QGVIcon* item = nullptr;
    QGV::GeoPos pos = QGV::GeoPos(0, 0);
    double angle = 0;
  };

  friend class GeoViewUiComposer;
  friend class GeoViewContourFeature;
  friend class GeoViewRouteFeature;
  friend class GeoViewRobotFeature;
  friend class GeoViewCommandFeature;
  friend class GeoViewCutoutFeature;
  friend class GeoViewInteractionFeature;

 public:
  GeoViewWidget(QWidget* parent = nullptr);
  ~GeoViewWidget();

  QGV::GeoRect targetArea() const;

  QGroupBox* createOptionsList();
  QGroupBox* createInfoList();

  void preloadImages();
  void loadImage(QImage& dest, QUrl url);

  void addRobot(double latitude, double longitude, double angle);
  void updateRobot(double latitude, double longitude, double angle);

  void updateInfoList();

  void buildRouteWithAngleForCustomRoute(
      double stepMeters, double angleDegrees, double offsetFromContour,
      const QGV::GeoPos& startPointParam, const QGV::GeoPos& endPointParam, bool rightSide,
      bool debugMode = false,
      RouteAlgo::PipelineStage visualizationStage = RouteAlgo::PipelineStage::Approach);
  // Re-renders the route layer with the given pipeline stage overlay using
  // the previously computed pipeline state. Cheap (no recomputation).
  // `stepLimit < 0` shows everything; otherwise progression inside the stage
  // is truncated for step-by-step playback.
  void setRouteVisualizationStage(RouteAlgo::PipelineStage stage, int stepLimit = -1);

  void handleMapClick(const QGV::GeoPos& pos);
  void handleMapClickStartRoutePoint(const QGV::GeoPos& pos);
  void onMapMousePress(QPointF projPos);
  void handleCutoutPolygonClick(const QGV::GeoPos& pos);
  void handleDrawContourClick(const QGV::GeoPos& pos);
  void cancelDrawContour();

  QPointF moveAlongContour(const ClipperLib::Path& contour,
                           const QPointF& startPt, double stepMeters,
                           bool forward);

  void drawRoute(QGVLayer* layer, const QVector<QGV::GeoPos>& pts,
                 const QPen& pen = QPen(QColor(220, 60, 60, 180), 2),
                 bool drawArrow = false, bool replaceExisting = false);

  QJsonDocument getRouteCommands() const;
  const RobotItem& getRobotItem() const;

 protected:
  std::optional<QGV::GeoPos> segmentIntersection(const QGV::GeoPos& a,
                                                 const QGV::GeoPos& b,
                                                 const QGV::GeoPos& c,
                                                 const QGV::GeoPos& d);

  QVector<QGV::GeoPos> polygonSelfIntersections(
      const QVector<QGV::GeoPos>& points);

  QJsonDocument generateGazeboJson();
  QJsonDocument generateGazeboWaypointsJson();

  inline void clearMainLayer() {
    if (mLayer)
      mLayer->deleteItems();
  }
  void clearRouteCommands();
  void setInteractionMode(InteractionMode mode);
  InteractionMode interactionMode() const;
  void setUiStatus(const QString& text, UiStatusLevel level = UiStatusLevel::Info);

  void clearRouteLayer();
  void clearRobotLayer();
  void clearCutouts();
  void redrawCutouts();
  void redrawActiveCutout();
  bool finishCutoutPolygon();

 private:
  void initMap();
  void initLayers();
  void initUi(QVBoxLayout* rootLayout);
  void connectSignals();
  void scheduleInitialCamera();

  double haversineDistance(const QGV::GeoPos& pos1, const QGV::GeoPos& pos2);
  double calculateBearing(const QGV::GeoPos& start, const QGV::GeoPos& end);
  double calculateRosYaw(const QGV::GeoPos& start, const QGV::GeoPos& end);
  QPointF computeGazeboPoint(const QGV::GeoPos& start, const QGV::GeoPos& end);
  QWidget* createRouteParamsWidget();
 signals:
  void routeBuilt(const QJsonDocument& jsonDoc);

 public slots:
  void addContour();
  void createRoute();
  void toggleManualRouteMode();
  void startCutoutPolygonMode();
  void startDrawContourOnMap();
  void undoLastCutout();
  void removeContour();
  void showRobotCommandsJson();

  void clearAll();

 private:
  QGVMap* mMap;

  // Centralized layer ownership/creation (see GeoViewMapLayers).
  GeoViewMapLayers mLayers;

  // Backward-compatible aliases used across the codebase (features, helpers).
  // These pointers are owned by QGeoView via mMap->addItem(...) and are created in initLayers().
  QGVLayer* mLayer = nullptr;
  QGVLayer* mRouteLayer = nullptr;
  QGVLayer* mCutoutLayer = nullptr;
  QGVLayer* mCutoutDraftLayer = nullptr;
  QGVLayer* mRobotLayer = nullptr;

  Contour* mContour = nullptr;
  ContourPreviewDialog* mPrevDialog = nullptr;

  QVector<QVector<QGV::GeoPos>> mCutoutPolygons;
  QVector<QGV::GeoPos> mActiveCutoutPolygon;
  QVector<QGV::GeoPos> mActiveContourPolygon;
  QVector<QVector<QGV::GeoPos>> mSplitRegionPreview;

  QPen mContourColor = QPen(QColor(220, 60, 60, 180), 3);
  QPen mRouteColor = QPen(QColor(50, 120, 255, 200), 2);

  /// Full-height right column: page 0 = controls + Info, page 1 = route params.
  QStackedWidget* mSidePanelStack = nullptr;
  QListWidget* mInfoWidget = nullptr;
  QLabel* mStatusLabel = nullptr;
  QWidget* mParamsWidget = nullptr;

  RobotItem mRobotItem;

  InteractionMode mInteractionMode = InteractionMode::Idle;
  RouteAlgo::PipelineStage mRouteVisualizationStage = RouteAlgo::PipelineStage::Approach;

  QImage mRobotIcon;
  QImage mArrow;

  QPushButton* mCreateRouteButton = nullptr;
  QPushButton* mCancelDrawContourButton = nullptr;

  void refreshContourDrawingUi();

  // Transitional shared state container used by extracted features.
  GeoViewState mState;
};
