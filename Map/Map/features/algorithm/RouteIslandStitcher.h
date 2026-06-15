#pragma once

#include "../RouteAlgorithmConfig.h"
#include <QPointF>
#include <QVector>
#include <functional>
#include <vector>

namespace RouteAlgo {

struct IslandRoute {
  QVector<QPointF> pts;
  QPointF head;
  QPointF tail;
  QPointF centroid;
  int id = -1;
  int compId = -1;
  int phase = 0;
  int chunkCount = 0;
  int minRow = -1;
  int maxRow = -1;
};

struct IslandLink {
  int from = -1;
  int to = -1;
  bool merged = false;
  QPointF fromPoint;
  QPointF toPoint;
};

struct StitchResult {
  QVector<QPointF> stitchedProj;
  std::vector<IslandLink> islandLinks;
  std::vector<int> transitionPointOffsets;
  std::vector<int> unreachableIslandIndices;
  int usedIslandCount = 0;
};

struct StitchParams {
  QPointF nearestPt;
  QPointF lineDir;
  QPointF normal;
  double stepMeters = 0.0;
  int componentCount = 0;
  double intraStrict = 0.0;  // small-island entry threshold
  double invalidConnectorCost = 1e12;
  bool routeDebug = false;
  RouteAlgorithmConfig config;
  std::function<double(const QPointF&, const QPointF&)> connectorCost;
  std::function<bool(QVector<QPointF>&, const QPointF&, const QPointF)> appendConnector;
};

StitchResult stitchIslands(const std::vector<IslandRoute>& islands, const StitchParams& p);

}  // namespace RouteAlgo
