#pragma once
 
class GeoViewWidget;
class QPointF;
 
// Обработка кликов и режима взаимодействия с картой.
// Вынесено из GeoViewWidget для декомпозиции (поведение должно остаться прежним).
class GeoViewInteractionFeature {
 public:
  static void onMapMousePress(GeoViewWidget& view, QPointF projPos);
};
