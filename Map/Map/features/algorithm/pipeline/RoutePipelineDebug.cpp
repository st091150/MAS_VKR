#include "RoutePipelineDebug.h"

#include "../../../utils/MapLog.h"
#include <QDebug>

namespace RouteAlgo {

QString pipelineStageName(PipelineStage stage) {
  switch (stage) {
    case PipelineStage::Prepare:  return QStringLiteral("Prepare");
    case PipelineStage::Sweep:    return QStringLiteral("Sweep");
    case PipelineStage::Filter:   return QStringLiteral("Filter");
    case PipelineStage::Graph:    return QStringLiteral("Graph");
    case PipelineStage::Merge:    return QStringLiteral("Merge");
    case PipelineStage::Routes:   return QStringLiteral("Routes");
    case PipelineStage::Group:    return QStringLiteral("Group");
    case PipelineStage::Stitch:   return QStringLiteral("Stitch");
    case PipelineStage::Approach: return QStringLiteral("Approach");
    case PipelineStage::Count:    break;
  }
  return QStringLiteral("Unknown");
}

QString stageStatusLabel(StageStatus status) {
  switch (status) {
    case StageStatus::Pending: return QStringLiteral("ожидание");
    case StageStatus::Running: return QStringLiteral("выполняется");
    case StageStatus::Ok:      return QStringLiteral("ok");
    case StageStatus::Failed:  return QStringLiteral("ошибка");
    case StageStatus::Skipped: return QStringLiteral("пропущена");
  }
  return QStringLiteral("?");
}

RoutePipelineDebug::RoutePipelineDebug() {
  mStages.reserve(static_cast<size_t>(PipelineStage::Count));
}

void RoutePipelineDebug::beginStage(PipelineStage stage, const QString& displayName) {
  StageInfo info;
  info.stage = stage;
  info.name = displayName.isEmpty() ? pipelineStageName(stage) : displayName;
  info.status = StageStatus::Running;
  mStages.push_back(std::move(info));
  mCurrentIndex = static_cast<int>(mStages.size()) - 1;
  mTimer.restart();
}

StageInfo& RoutePipelineDebug::currentStage() {
  Q_ASSERT(mCurrentIndex >= 0 && mCurrentIndex < static_cast<int>(mStages.size()));
  return mStages[static_cast<size_t>(mCurrentIndex)];
}

void RoutePipelineDebug::log(const QString& message) {
  if (mCurrentIndex < 0) return;
  currentStage().log.push_back(message);
  if (mConsoleEnabled) {
    qCDebug(logPipeline).noquote()
        << QStringLiteral("[%1] %2").arg(currentStage().name, message);
  }
}

void RoutePipelineDebug::metric(const QString& key, const QString& value) {
  if (mCurrentIndex < 0) return;
  currentStage().metrics.push_back({key, value});
}

void RoutePipelineDebug::metric(const QString& key, int value) {
  metric(key, QString::number(value));
}

void RoutePipelineDebug::metric(const QString& key, qsizetype value) {
  metric(key, QString::number(static_cast<long long>(value)));
}

void RoutePipelineDebug::metric(const QString& key, double value, int decimals) {
  metric(key, QString::number(value, 'f', decimals));
}

void RoutePipelineDebug::summarize(const QString& message) {
  if (mCurrentIndex < 0) return;
  currentStage().message = message;
}

void RoutePipelineDebug::endStage(StageStatus status) {
  if (mCurrentIndex < 0) return;
  StageInfo& info = currentStage();
  info.status = status;
  info.elapsedMs = mTimer.elapsed();
  mCurrentIndex = -1;
}

void RoutePipelineDebug::fail(const QString& message) {
  if (mCurrentIndex < 0) return;
  currentStage().message = message;
  currentStage().log.push_back(QStringLiteral("FAIL: %1").arg(message));
  endStage(StageStatus::Failed);
}

const StageInfo* RoutePipelineDebug::find(PipelineStage stage) const {
  for (const auto& s : mStages) {
    if (s.stage == stage) return &s;
  }
  return nullptr;
}

bool RoutePipelineDebug::stageOk(PipelineStage stage) const {
  const auto* s = find(stage);
  return s && s->status == StageStatus::Ok;
}

const StageInfo* RoutePipelineDebug::firstFailure() const {
  for (const auto& s : mStages) {
    if (s.status == StageStatus::Failed) return &s;
  }
  return nullptr;
}

}  // namespace RouteAlgo
