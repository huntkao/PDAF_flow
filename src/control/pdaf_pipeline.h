#pragma once
#include <pdaf/algo/depth_estimator.h>
#include <pdaf/algo/pd_cost_engine.h>
#include <pdaf/control/focus_estimator.h>

#include <memory>

namespace pdaf {
// M1+M2 組合；hw_costs 存在時走 HW 統計路徑（M1 被 ISP 硬體取代的情境）
class PdafPipeline : public IFocusEstimator {
 public:
  PdafPipeline(std::unique_ptr<IPdCostEngine> cost_engine,
               std::unique_ptr<IDepthEstimator> depth_estimator);
  std::vector<DepthEstimate> process(const PdInput& input) override;

 private:
  std::unique_ptr<IPdCostEngine> cost_engine_;
  std::unique_ptr<IDepthEstimator> depth_estimator_;
};
}  // namespace pdaf
