#include "Stage3Filter.h"

#include "RoutePipelineDebug.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace RouteAlgo {

namespace {
// "Relaxed length" threshold: stripes shorter than the strict minimum are
// still kept if they bridge a real gap (have neighbors above and below).
constexpr double kRelaxedKeepShareOfMin = 0.55;
// Hard floor for "neighbors must overlap by at least this much in along-line
// direction" (in step units).
constexpr double kNeighborOverlapTolMeters = 0.4;
constexpr double kNeighborOverlapTolStepFactor = 0.12;
// Tolerance used when collapsing very close `d` values onto a single row.
constexpr double kRowMergeEpsilonMeters = 0.05;
constexpr double kRowMergeEpsilonStepFactor = 0.05;
}  // namespace

bool runStageFilter(const RoutePipelineInput& input, RoutePipelineState& state,
                    RoutePipelineDebug& debug) {
  debug.beginStage(PipelineStage::Filter);

  std::vector<StripeSegment> kept;
  std::vector<StripeSegment> dropped;
  kept.reserve(state.rawStripes.size());
  const double minLen = state.tuning.minUsefulSegmentLength;
  const double relaxedLen = minLen * kRelaxedKeepShareOfMin;

  auto rangeOnLine = [&](const StripeSegment& s) {
    const double ta = QPointF::dotProduct(s.a, state.lineDir);
    const double tb = QPointF::dotProduct(s.b, state.lineDir);
    return std::pair<double, double>(std::min(ta, tb), std::max(ta, tb));
  };
  const double overlapTol =
      std::max(kNeighborOverlapTolMeters, input.stepMeters * kNeighborOverlapTolStepFactor);

  // IMPORTANT: `StripeSegment::row` at this point is the sweep traversal order
  // (0, +1, +2, ..., -1, -2, ...). It is NOT the physical neighbor relation by `d`.
  // Neighbor support must be evaluated in `d`-space.
  const double mergeEpsRaw =
      std::max(kRowMergeEpsilonMeters, input.stepMeters * kRowMergeEpsilonStepFactor);
  std::vector<double> uniqueDsRaw;
  uniqueDsRaw.reserve(state.rawStripes.size());
  for (const auto& s : state.rawStripes) uniqueDsRaw.push_back(s.d);
  std::sort(uniqueDsRaw.begin(), uniqueDsRaw.end());
  uniqueDsRaw.erase(std::unique(uniqueDsRaw.begin(), uniqueDsRaw.end(),
                                [mergeEpsRaw](double a, double b) {
                                  return std::abs(a - b) < mergeEpsRaw;
                                }),
                    uniqueDsRaw.end());
  auto rowOfDRaw = [&](double d) {
    auto it = std::lower_bound(uniqueDsRaw.begin(), uniqueDsRaw.end(), d - mergeEpsRaw);
    int idx = static_cast<int>(it - uniqueDsRaw.begin());
    if (idx > 0 && idx < static_cast<int>(uniqueDsRaw.size()) &&
        std::abs(uniqueDsRaw[idx - 1] - d) < std::abs(uniqueDsRaw[idx] - d)) {
      --idx;
    }
    return std::clamp(idx, 0, static_cast<int>(uniqueDsRaw.size()) - 1);
  };

  auto hasNeighborSupport = [&](const StripeSegment& s) {
    bool prevOk = false;
    bool nextOk = false;
    const auto rs = rangeOnLine(s);
    const int baseRow = rowOfDRaw(s.d);
    for (const auto& other : state.rawStripes) {
      if (&other == &s) continue;
      const int dr = rowOfDRaw(other.d) - baseRow;
      if (std::abs(dr) != 1) continue;
      const auto ro = rangeOnLine(other);
      const double overlap = std::min(rs.second, ro.second) - std::max(rs.first, ro.first);
      if (overlap < -overlapTol) continue;
      if (dr < 0) prevOk = true;
      else nextOk = true;
      if (prevOk && nextOk) return true;
    }
    return false;
  };

  int droppedShort = 0;
  for (const auto& s : state.rawStripes) {
    const double len = std::hypot((s.b - s.a).x(), (s.b - s.a).y());
    if (len < minLen) {
      const bool gapFill = (len >= relaxedLen) && hasNeighborSupport(s);
      if (gapFill) {
        kept.push_back(s);
      } else {
        ++droppedShort;
        dropped.push_back(s);
      }
      continue;
    }
    kept.push_back(s);
  }

  if (kept.empty()) {
    state.stripes.clear();
    state.droppedStripes = std::move(dropped);
    state.rowCount = 0;
    state.rowsSegments.clear();
    debug.metric(QStringLiteral("input"), static_cast<int>(state.rawStripes.size()));
    debug.metric(QStringLiteral("kept"), 0);
    debug.metric(QStringLiteral("dropped"), droppedShort);
    debug.fail(QStringLiteral("Не найдено валидных сегментов покрытия"));
    return false;
  }

  // Re-index `row` so that row+1 is always the next physical stripe by `d`.
  // Sweep produces rows in traversal order (0, +1, +2, ..., -1, -2, ...),
  // which makes naive "row+1" jump across the polygon — a row of bugs.
  std::vector<double> uniqueDs;
  uniqueDs.reserve(kept.size());
  for (const auto& s : kept) uniqueDs.push_back(s.d);
  std::sort(uniqueDs.begin(), uniqueDs.end());
  const double mergeEpsKept =
      std::max(kRowMergeEpsilonMeters, input.stepMeters * kRowMergeEpsilonStepFactor);
  uniqueDs.erase(std::unique(uniqueDs.begin(), uniqueDs.end(),
                              [mergeEpsKept](double a, double b) {
                                return std::abs(a - b) < mergeEpsKept;
                              }),
                  uniqueDs.end());
  auto rowOfD = [&](double d) {
    auto it = std::lower_bound(uniqueDs.begin(), uniqueDs.end(), d - mergeEpsKept);
    int idx = static_cast<int>(it - uniqueDs.begin());
    if (idx > 0 && idx < static_cast<int>(uniqueDs.size()) &&
        std::abs(uniqueDs[idx - 1] - d) < std::abs(uniqueDs[idx] - d)) {
      --idx;
    }
    return std::clamp(idx, 0, static_cast<int>(uniqueDs.size()) - 1);
  };
  for (auto& s : kept) s.row = rowOfD(s.d);
  const int rowCount = static_cast<int>(uniqueDs.size());

  std::sort(kept.begin(), kept.end(), [](const StripeSegment& a, const StripeSegment& b) {
    if (a.row != b.row) return a.row < b.row;
    return a.d < b.d;
  });

  std::vector<std::vector<StripeSegment>> rowsSegments(static_cast<size_t>(rowCount));
  for (const auto& seg : kept) {
    if (seg.row >= 0 && seg.row < rowCount) {
      rowsSegments[static_cast<size_t>(seg.row)].push_back(seg);
    }
  }

  state.stripes = std::move(kept);
  state.droppedStripes = std::move(dropped);
  state.rowCount = rowCount;
  state.rowsSegments = std::move(rowsSegments);

  debug.metric(QStringLiteral("input"), static_cast<int>(state.rawStripes.size()));
  debug.metric(QStringLiteral("kept"), static_cast<int>(state.stripes.size()));
  debug.metric(QStringLiteral("dropped"), droppedShort);
  debug.metric(QStringLiteral("rows"), state.rowCount);
  debug.metric(QStringLiteral("minLen, м"), minLen, 2);
  debug.summarize(QStringLiteral("Оставлено %1 из %2 полос; %3 строк")
                      .arg(static_cast<int>(state.stripes.size()))
                      .arg(static_cast<int>(state.rawStripes.size()))
                      .arg(state.rowCount));
  debug.endStage(StageStatus::Ok);
  return true;
}

}  // namespace RouteAlgo
