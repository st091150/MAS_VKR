#include "Stage1Prepare.h"

#include "RoutePipelineDebug.h"

#include "../../../utils/ClipperUtils.h"

#include <QtMath>
#include <cmath>
#include <memory>

namespace RouteAlgo {

namespace {
constexpr double kZeroDirEpsilon = 1e-6;

int countOuterContours(const ClipperLib::Paths& paths) {
  int count = 0;
  for (const auto& path : paths) {
    if (path.size() >= 3 && ClipperLib::Area(path) > 0.0) ++count;
  }
  return count;
}
}

bool runStagePrepare(const RoutePipelineInput& input, RoutePipelineState& state,
                     RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Prepare);

  if (input.projContour.size() < 2 || input.stepMeters <= 0.0) {
    debug.fail(QStringLiteral("Недостаточно данных: нужен контур и положительный шаг"));
    return false;
  }

  ClipperLib::Paths workingRegion;
  workingRegion.push_back(ClipperUtils::toClipPath(input.projContour));
  ClipperUtils::orientClipperPathOuter(workingRegion.back());

  ClipperLib::Paths cutoutPaths;
  cutoutPaths.reserve(input.projCutouts.size());
  for (const auto& cutout : input.projCutouts) {
    if (cutout.size() < 3) continue;
    ClipperLib::Path cut = ClipperUtils::toClipPath(cutout);
    ClipperUtils::orientClipperPathOuter(cut);
    cutoutPaths.push_back(std::move(cut));
  }
  if (!cutoutPaths.empty()) {
    workingRegion = ClipperUtils::difference(ClipperUtils::unionPaths(workingRegion),
                                             ClipperUtils::unionPaths(cutoutPaths));
  } else {
    workingRegion = ClipperUtils::unionPaths(workingRegion);
  }
  if (workingRegion.empty()) {
    debug.fail(QStringLiteral("Пустая рабочая область после вырезов"));
    return false;
  }

  state.workingRegion = workingRegion;
  state.workingGeo = buildRegionGeometry(workingRegion);

  ClipperLib::ClipperOffset co;
  co.AddPaths(workingRegion, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);

  ClipperLib::Paths insetRaw;
  co.Execute(insetRaw, -input.contourOffset * ClipperUtils::kClipperScale);
  state.insetRegion = ClipperUtils::unionPaths(insetRaw);
  if (state.insetRegion.empty()) {
    debug.fail(QStringLiteral("Пустая рабочая область после внутреннего отступа"));
    return false;
  }

  state.stepMeters = input.stepMeters;
  state.insetGeo = buildRegionGeometry(state.insetRegion);
  state.nearestPt = nearestPointOnRegion(state.insetRegion, input.startProj);

  const double angleRad = input.angleDegrees * M_PI / 180.0;
  state.lineDir = normalizeOrZero(QPointF(std::sin(angleRad), -std::cos(angleRad)));
  if (std::hypot(state.lineDir.x(), state.lineDir.y()) < kZeroDirEpsilon) {
    debug.fail(QStringLiteral("Некорректное направление линий (угол=%1)")
                   .arg(input.angleDegrees, 0, 'f', 1));
    return false;
  }
  state.normal = QPointF(-state.lineDir.y(), state.lineDir.x());

  state.tuning.minUsefulSegmentLength =
      std::max(input.config.minUsefulSegmentLengthMin,
               input.stepMeters * input.config.minUsefulSegmentLengthFactor);
  state.tuning.intraStrict =
      std::max(input.config.intraStrictMin, input.stepMeters * input.config.intraStrictFactor);
  state.tuning.invalidConnectorCost = input.config.invalidConnectorCost;

  state.connectorPolicy = std::make_shared<RouteConnectorPolicy>(
      state.insetGeo, input.config, input.stepMeters, state.tuning.invalidConnectorCost,
      input.routeDebug);

  const int outerContours = countOuterContours(state.insetRegion);
  debug.metric(QStringLiteral("paths"), static_cast<int>(state.insetRegion.size()));
  debug.metric(QStringLiteral("outerContours"), outerContours);
  debug.metric(QStringLiteral("cutouts"), static_cast<int>(input.projCutouts.size()));
  int holeCount = 0;
  for (const auto& path : workingRegion) {
    if (path.size() >= 3 && ClipperLib::Area(path) < 0.0) ++holeCount;
  }
  debug.metric(QStringLiteral("holes"), holeCount);
  debug.metric(QStringLiteral("step, м"), input.stepMeters, 2);
  debug.metric(QStringLiteral("angle, °"), input.angleDegrees, 1);
  debug.metric(QStringLiteral("offset, м"), input.contourOffset, 2);
  debug.metric(QStringLiteral("minUsefulLen, м"), state.tuning.minUsefulSegmentLength, 2);
  debug.metric(QStringLiteral("intraStrict, м"), state.tuning.intraStrict, 2);
  if (outerContours > 1) {
    // ВАЖНО: `outerContours > 1` означает, что после вырезов/отступа область стала
    // состоять из нескольких независимых регионов. Пайплайн продолжает работу,
    // но далее (стадии Sweep/Graph/...) фактически строят покрытие для каждого
    // региона отдельно и могут дать "рваный" результат для одной большой задачи.
    debug.log(QStringLiteral("Получен мультиконтур: %1 независимых рабочих областей")
                  .arg(outerContours));
    debug.summarize(QStringLiteral("Внимание: рабочая область разорвана на %1 контуров")
                        .arg(outerContours));
  } else {
    debug.summarize(QStringLiteral("Подготовлена рабочая область (%1 контуров)")
                        .arg(static_cast<int>(state.insetRegion.size())));
  }
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
