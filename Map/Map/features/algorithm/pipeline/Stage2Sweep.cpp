#include "Stage2Sweep.h"

#include "RoutePipelineDebug.h"

#include "../../../utils/ClipperUtils.h"

#include <QPointF>
#include <QVector>

#include <algorithm>
#include <cmath>
#include <limits>

namespace RouteAlgo {

namespace {
// Hard upper bound on coverage passes: protects against degenerate residuals
// that never shrink. 6 is well above what any sensible polygon needs.
constexpr int kMaxCoveragePasses = 6;
// Minimum residual area considered "still worth sweeping" (in step units²).
constexpr double kResidualAreaStepFactor = 0.5;
constexpr double kMinResidualAreaMeters2 = 1.0;
// Sweep "band" half-width as a fraction of step (band = swept stripe corridor).
constexpr double kSweepBandHalfStepFactor = 0.5;
constexpr double kSweepBandHalfMin = 0.25;
// Length of the line we use for cutting the band out of the residual region.
constexpr double kSweepLineHalfMin = 20.0;
constexpr double kSweepLineHalfDiagFactor = 1.5;
// Hard cap on "duplicate d" detection to avoid double-counting close rows.
constexpr double kDuplicateDistanceMeters = 0.05;
// We bail out if N consecutive passes don't shrink the residual significantly.
constexpr int kStalledPassesLimit = 2;
constexpr double kStalledDeltaShareOfMin = 0.1;
}  // namespace

bool runStageSweep(const RoutePipelineInput& input, RoutePipelineState& state,
                   RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Sweep);

  ClipperLib::Paths remainingRegion = state.insetRegion;
  std::vector<StripeSegment> stripes;
  int row = 0;
  int passesExecuted = 0;
  const double minResidualArea =
      std::max(kMinResidualAreaMeters2,
               input.stepMeters * input.stepMeters * kResidualAreaStepFactor);
  double prevRemainingArea = std::abs(regionSignedArea(remainingRegion));
  int stalledPasses = 0;

  for (int pass = 0; pass < kMaxCoveragePasses; ++pass) {
    if (remainingRegion.empty()) break;
    const RegionGeometry passGeo = buildRegionGeometry(remainingRegion);
    const QPointF passAnchorPt = nearestPointOnRegion(remainingRegion, input.startProj);
    double minD = std::numeric_limits<double>::max();
    double maxD = -std::numeric_limits<double>::max();
    for (const auto& path : remainingRegion) {
      for (const auto& cp : path) {
        const QPointF p = ClipperUtils::fromClip(cp);
        const double d = QPointF::dotProduct(p, state.normal);
        minD = std::min(minD, d);
        maxD = std::max(maxD, d);
      }
    }
    const double anchorD = QPointF::dotProduct(passAnchorPt, state.normal);
    const int kPos =
        (maxD >= anchorD) ? static_cast<int>(std::floor((maxD - anchorD) / input.stepMeters)) : 0;
    const int kNeg =
        (anchorD >= minD) ? static_cast<int>(std::floor((anchorD - minD) / input.stepMeters)) : 0;

    QVector<int> kSequence;
    kSequence.push_back(0);
    auto appendSide = [&](int sign) {
      const int limit = (sign > 0) ? kPos : kNeg;
      for (int i = 1; i <= limit; ++i) {
        kSequence.push_back(sign * i);
      }
    };
    // Side preferred by the user goes first so the very first island is built
    // in the requested direction; the opposite side then guarantees coverage.
    appendSide(input.rightSide ? 1 : -1);
    appendSide(input.rightSide ? -1 : 1);

    std::vector<double> passDs;
    int localRows = 0;
    int passSegs = 0;
    for (int k : kSequence) {
      const double d = anchorD + static_cast<double>(k) * input.stepMeters;
      const QPointF linePoint = state.normal * d;
      std::vector<StripeSegment> rowSegs =
          intersectInfiniteLineWithRegion(passGeo, linePoint, state.lineDir, d, row + localRows);
      if (!rowSegs.empty()) {
        bool hasD = false;
        for (double existing : passDs) {
          if (std::abs(existing - d) < kDuplicateDistanceMeters) {
            hasD = true;
            break;
          }
        }
        if (!hasD) passDs.push_back(d);
      }
      passSegs += static_cast<int>(rowSegs.size());
      for (auto& s : rowSegs) {
        stripes.push_back(std::move(s));
      }
      ++localRows;
    }
    row += localRows;

    debug.log(QStringLiteral("pass=%1: rows=%2, segments=%3 (kPos=%4, kNeg=%5)")
                  .arg(pass)
                  .arg(localRows)
                  .arg(passSegs)
                  .arg(kPos)
                  .arg(kNeg));
    if (passSegs == 0 || passDs.empty()) break;

    const RegionBounds b = computeRegionBounds(remainingRegion);
    const double diag = std::hypot(b.maxX - b.minX, b.maxY - b.minY);
    const double lineHalf = std::max(kSweepLineHalfMin, diag * kSweepLineHalfDiagFactor);
    const double bandHalf =
        std::max(kSweepBandHalfMin, input.stepMeters * kSweepBandHalfStepFactor);
    // ВАЖНО (геометрия):
    // Полоса "сметания" строится как прямоугольник вокруг каждой sweep-линии.
    // Если якорить её в точке `normal * d`, прямоугольник может оказаться
    // далеко от реального полигона (особенно в Web Mercator с большими координатами),
    // и пересечение с регионом станет пустым. Поэтому якорим в центре bbox региона
    // и сдвигаем вдоль normal до нужного `d`.
    const QPointF bboxCenter((b.minX + b.maxX) * 0.5, (b.minY + b.maxY) * 0.5);
    ClipperLib::Paths coveredPass;
    for (double d : passDs) {
      const double centerShift = d - QPointF::dotProduct(bboxCenter, state.normal);
      const QPointF c = bboxCenter + state.normal * centerShift;
      QVector<QPointF> bandPts{
          c - state.lineDir * lineHalf - state.normal * bandHalf,
          c + state.lineDir * lineHalf - state.normal * bandHalf,
          c + state.lineDir * lineHalf + state.normal * bandHalf,
          c - state.lineDir * lineHalf + state.normal * bandHalf};
      ClipperLib::Path band = ClipperUtils::toClipPath(bandPts, true);
      ClipperLib::Paths inter = intersectRegionWithBand(remainingRegion, band);
      if (!inter.empty()) {
        coveredPass.insert(coveredPass.end(), inter.begin(), inter.end());
      }
    }
    coveredPass = ClipperUtils::unionPaths(coveredPass);
    if (coveredPass.empty()) break;

    remainingRegion = ClipperUtils::difference(remainingRegion, coveredPass);
    remainingRegion = ClipperUtils::unionPaths(remainingRegion);
    const double areaNow = std::abs(regionSignedArea(remainingRegion));
    debug.log(QStringLiteral("pass=%1: residualArea=%2, deltaArea=%3")
                  .arg(pass)
                  .arg(areaNow, 0, 'f', 2)
                  .arg(prevRemainingArea - areaNow, 0, 'f', 2));
    ++passesExecuted;
    const double deltaArea = (prevRemainingArea - areaNow);
    if (areaNow <= minResidualArea) break;
    if (deltaArea < minResidualArea * kStalledDeltaShareOfMin) {
      ++stalledPasses;
    } else {
      stalledPasses = 0;
    }
    if (stalledPasses >= kStalledPassesLimit) break;
    prevRemainingArea = areaNow;
  }

  state.rawStripes = std::move(stripes);
  state.sweepRowCount = row;
  state.passesExecuted = passesExecuted;
  state.residualRegion = std::move(remainingRegion);

  debug.metric(QStringLiteral("rawStripes"), static_cast<int>(state.rawStripes.size()));
  debug.metric(QStringLiteral("rows"), state.sweepRowCount);
  debug.metric(QStringLiteral("passes"), state.passesExecuted);
  debug.metric(QStringLiteral("residualPaths"),
               static_cast<int>(state.residualRegion.size()));
  if (state.rawStripes.empty()) {
    debug.fail(QStringLiteral("Не удалось построить покрывающие полосы"));
    return false;
  }
  debug.summarize(QStringLiteral("Сметание построило %1 полос за %2 проходов")
                      .arg(static_cast<int>(state.rawStripes.size()))
                      .arg(state.passesExecuted));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
