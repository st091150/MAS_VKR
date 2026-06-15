#pragma once
 
#include <QGeoView/QGVLayer.h>
#include <QGeoView/QGVMap.h>
 
// Небольшой контейнер для стандартных слоёв GeoView.
// Цель: убрать "простыню" initLayers() из GeoViewWidget и централизовать имена/создание.
struct GeoViewMapLayers {
  QGVLayer* main = nullptr;
  QGVLayer* route = nullptr;
  QGVLayer* cutout = nullptr;
  QGVLayer* cutoutDraft = nullptr;
  QGVLayer* robot = nullptr;
 
  static GeoViewMapLayers createAndAttach(QGVMap& map) {
    GeoViewMapLayers out;
 
    out.main = new QGVLayer();
    out.main->setName("MainLayer");
 
    out.route = new QGVLayer();
    out.route->setName("RouteLayer");
 
    out.cutout = new QGVLayer();
    out.cutout->setName("CutoutLayer");
 
    out.cutoutDraft = new QGVLayer();
    out.cutoutDraft->setName("CutoutDraftLayer");

    out.robot = new QGVLayer();
    out.robot->setName("RobotLayer");
 
    map.addItem(out.main);
    map.addItem(out.cutout);
    map.addItem(out.cutoutDraft);
    map.addItem(out.route);
    map.addItem(out.robot);
 
    return out;
  }
};
