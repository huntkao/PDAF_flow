#pragma once
#include <pdaf/types.h>

#include <vector>

namespace pdaf {

struct DccAnchor {
  int step = 0;
  float steps_per_disparity = 50.f;  // DCC：1 單位 disparity 對應的 VCM step 數
};

struct DccTable {
  int step_min = 0;
  int step_max = 1023;
  std::vector<DccAnchor> anchors;  // 依 step 遞增排序，至少 1 個
};

// 錨點間線性內插、兩端 clamp（模擬器共用，確保閉環一致）
float dccInterp(const DccTable& table, int step);

// M3：套用 DCC 校正，輸出 VCM step 命令；基準為曝光當下的 lens 位置
class ILensMapper {
 public:
  virtual ~ILensMapper() = default;
  virtual void init(const DccTable& table) = 0;
  virtual LensCommand toLensCommand(const DepthEstimate& est,
                                    int lens_step_at_exposure) = 0;
};

}  // namespace pdaf
