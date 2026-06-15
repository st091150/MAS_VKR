#pragma once

class QGroupBox;
class QWidget;
class GeoViewWidget;

class GeoViewUiComposer {
 public:
  static QGroupBox* createOptionsList(GeoViewWidget& view);
  static QGroupBox* createInfoList(GeoViewWidget& view);
  static QWidget* createRouteParamsWidget(GeoViewWidget& view);
};
