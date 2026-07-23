#pragma once
#include <pdaf/types.h>

#include <vector>

namespace pdaf {
// 對焦估測技術的抽象（HAF 式仲裁縫）：PDAF 是其中一種實作，
// 將來 contrast/TOF 也以此介面掛入，controller 不需修改。
class IFocusEstimator {
 public:
  virtual ~IFocusEstimator() = default;
  virtual std::vector<DepthEstimate> process(const PdInput& input) = 0;
};
}  // namespace pdaf
