#pragma once
#include <pdaf/types.h>

namespace pdaf
{
// HAL：phase pixel 資料來源（模擬器 / dump 重播 / 真硬體）
class IPdDataSource
{
public:
  virtual ~IPdDataSource() = default;
  virtual PdInput capture(const AfRequest& request) = 0;
};
} // namespace pdaf
