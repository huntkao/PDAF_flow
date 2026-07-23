#pragma once
#include <pdaf/algo/lens_mapper.h>

namespace pdaf
{
class DccLensMapper : public ILensMapper
{
public:
  void init(const DccTable& table) override;
  LensCommand toLensCommand(const DepthEstimate& est,
                            int lens_step_at_exposure) override;

private:
  DccTable table_;
};
} // namespace pdaf
