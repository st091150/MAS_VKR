#include "RoutePipeline.h"

#include "Stage1Prepare.h"
#include "Stage2Sweep.h"
#include "Stage3Filter.h"
#include "Stage4Graph.h"
#include "Stage5Merge.h"
#include "Stage6Routes.h"
#include "Stage7Group.h"
#include "Stage8Stitch.h"
#include "Stage9Approach.h"

namespace RouteAlgo {

RoutePipeline::RoutePipeline() {
  mState = RoutePipelineState{};
  mDebug = RoutePipelineDebug{};
}

bool RoutePipeline::run(const RoutePipelineInput& input) {
  mState = RoutePipelineState{};
  mDebug = RoutePipelineDebug{};
  mDebug.setConsoleEnabled(input.routeDebug);

  using StageFn = bool (*)(const RoutePipelineInput&, RoutePipelineState&, RoutePipelineDebug&);
  struct StageEntry {
    PipelineStage stage;
    StageFn fn;
  };
  const StageEntry stages[] = {
      {PipelineStage::Prepare,  &runStagePrepare},
      {PipelineStage::Sweep,    &runStageSweep},
      {PipelineStage::Filter,   &runStageFilter},
      {PipelineStage::Graph,    &runStageGraph},
      {PipelineStage::Merge,    &runStageMerge},
      {PipelineStage::Routes,   &runStageRoutes},
      {PipelineStage::Group,    &runStageGroup},
      {PipelineStage::Stitch,   &runStageStitch},
      {PipelineStage::Approach, &runStageApproach},
  };

  mSucceeded = false;
  mLastSuccessful = PipelineStage::Prepare;
  for (const auto& entry : stages) {
    if (!entry.fn(input, mState, mDebug)) {
      mSucceeded = false;
      return false;
    }
    mLastSuccessful = entry.stage;
  }
  mSucceeded = true;
  return true;
}

}  // namespace RouteAlgo
