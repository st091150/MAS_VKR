#pragma once

#include "RoutePipelineTypes.h"

#include <QElapsedTimer>
#include <QPair>
#include <QString>
#include <QStringList>

#include <vector>

namespace RouteAlgo {

// ---------------------------------------------------------------------------
// Per-stage debug capture: status, free-form log lines, named metrics, time.
// ---------------------------------------------------------------------------
enum class StageStatus : int {
  Pending,
  Running,
  Ok,
  Failed,
  Skipped,
};

QString stageStatusLabel(StageStatus status);

struct StageInfo {
  PipelineStage stage = PipelineStage::Prepare;
  QString name;
  StageStatus status = StageStatus::Pending;
  QString message;          // single-line summary shown next to the status
  QStringList log;
  QList<QPair<QString, QString>> metrics;
  qint64 elapsedMs = 0;
};

class RoutePipelineDebug {
 public:
  RoutePipelineDebug();

  // Управляет только выводом в консоль/Debug Output.
  // Сбор логов в StageInfo::log остаётся всегда (нужен UI/визуализатору).
  void setConsoleEnabled(bool enabled) { mConsoleEnabled = enabled; }

  // Stage-level mutators (called by the orchestrator and individual stages).
  void beginStage(PipelineStage stage, const QString& displayName = QString());
  void log(const QString& message);
  void metric(const QString& key, const QString& value);
  void metric(const QString& key, int value);
  void metric(const QString& key, qsizetype value);
  void metric(const QString& key, double value, int decimals = 3);
  void summarize(const QString& message);
  void endStage(StageStatus status);
  void fail(const QString& message);

  // Read-only access used by UI / visualizer.
  const std::vector<StageInfo>& stages() const { return mStages; }
  const StageInfo* find(PipelineStage stage) const;
  bool stageOk(PipelineStage stage) const;
  // Last failed stage (or nullptr).
  const StageInfo* firstFailure() const;

 private:
  StageInfo& currentStage();

  std::vector<StageInfo> mStages;
  int mCurrentIndex = -1;
  QElapsedTimer mTimer;
  bool mConsoleEnabled = false;
};

}  // namespace RouteAlgo
