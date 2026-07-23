#pragma once
#include <pdaf/types.h>

#include <vector>

namespace pdaf
{
// 對焦估測技術的抽象（HAF 式仲裁縫）：PDAF 是其中一種實作。
// 這條縫對「其他 PD-based 估測器」是成立的，可直接掛入而 controller 不需修改；
// 但 contrast/TOF 屬於真正不同的模態，PdInput 目前只帶 PD 樣本/PD cost，
// 並無全幅 luma 或 ToF depth，因此要接上這類模態還需先擴充 PdInput 增加對應的資料分支。
class IFocusEstimator
{
public:
  virtual ~IFocusEstimator() = default;
  virtual std::vector<DepthEstimate> process(const PdInput& input) = 0;
};
} // namespace pdaf
