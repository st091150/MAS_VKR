#pragma once

#include "../RouteAlgorithmConfig.h"
#include "RouteGeometryUtils.h"
#include <QPointF>
#include <QVector>

namespace RouteAlgo {

enum class ConnectorHopKind { InterIsland, IntraIslandSnake };

class RouteConnectorPolicy {
 public:
  RouteConnectorPolicy(const RegionGeometry& region, const RouteAlgorithmConfig& config,
                       double stepMeters, double invalidConnectorCost, bool routeDebug);

  double connectorCost(const QPointF& from, const QPointF& to) const {
    return connectorCost(from, to, ConnectorHopKind::InterIsland);
  }
  double connectorCost(const QPointF& from, const QPointF& to, ConnectorHopKind kind) const;

  bool appendConnectorProj(QVector<QPointF>& projRoute, const QPointF& from, const QPointF& to) const {
    return appendConnectorProj(projRoute, from, to, ConnectorHopKind::InterIsland);
  }
  bool appendConnectorProj(QVector<QPointF>& projRoute, const QPointF& from, const QPointF& to,
                           ConnectorHopKind kind) const;
  bool isDirectInside(const QPointF& a, const QPointF& b) const;

 private:
  void chooseBestBoundaryTransfer(const QPointF& fromPoint, const QPointF& toPoint,
                                  BoundaryAnchor& bestFrom, BoundaryAnchor& bestTo,
                                  double& bestScore) const;
  double directLengthCap(ConnectorHopKind kind) const;

  const RegionGeometry& mRegion;
  RouteAlgorithmConfig mConfig;
  double mStepMeters = 0.0;
  double mInvalidConnectorCost = 1e12;
  bool mRouteDebug = false;
};

}  // namespace RouteAlgo
