#pragma once

#include "RoutePipelineDebug.h"
#include "RoutePipelineTypes.h"

namespace RouteAlgo {

// ---------------------------------------------------------------------------
// Orchestrator
// ---------------------------------------------------------------------------
// Runs the 9 algorithm stages in fixed order, threading a single state and
// debug capture through them. If a stage returns false, the pipeline stops
// and `lastSuccessfulStage()` reports how far it got.
class RoutePipeline {
 public:
  RoutePipeline();

  // Run all stages in order. Returns true if every stage succeeded.
  bool run(const RoutePipelineInput& input);

  const RoutePipelineState& state() const { return mState; }
  RoutePipelineState& state() { return mState; }
  const RoutePipelineDebug& debug() const { return mDebug; }
  PipelineStage lastSuccessfulStage() const { return mLastSuccessful; }
  bool succeeded() const { return mSucceeded; }

 private:
  RoutePipelineState mState;
  RoutePipelineDebug mDebug;
  PipelineStage mLastSuccessful = PipelineStage::Prepare;
  bool mSucceeded = false;
};

}  // namespace RouteAlgo
