#pragma once
#include <pdaf/types.h>

namespace pdaf
{
// HAL：VCM actuator
class ILensActuator
{
public:
  virtual ~ILensActuator() = default;
  virtual void moveTo(int step) = 0;
  virtual LensStatus getStatus() const = 0;
};
} // namespace pdaf
