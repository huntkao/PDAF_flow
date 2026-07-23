#include "control/pdaf_pipeline.h"

namespace pdaf
{

PdafPipeline::PdafPipeline(std::unique_ptr<IPdCostEngine> cost_engine,
                           std::unique_ptr<IDepthEstimator> depth_estimator)
    : cost_engine_(std::move(cost_engine)),
      depth_estimator_(std::move(depth_estimator))
{
}

std::vector<DepthEstimate> PdafPipeline::process(const PdInput& input)
{
  std::vector<CostSequence> costs;
  if (input.hw_costs)
  {
    costs = *input.hw_costs;
  }
  else if (input.raw)
  {
    costs = cost_engine_->compute(*input.raw);
  }

  std::vector<DepthEstimate> out;
  out.reserve(costs.size());
  for (const auto& c : costs)
  {
    out.push_back(depth_estimator_->estimate(c));
  }
  return out;
}

} // namespace pdaf
