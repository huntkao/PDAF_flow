#pragma once
#include <pdaf/hal/pd_data_source.h>

#include <string>
#include <vector>

namespace pdaf {
// 讀取 dir 下 frame_0000.json 起的連號 dump，依序回放；放完重複最後一筆。
// 用途：單模塊驗證、真機資料離線分析。
class ReplayPdDataSource : public IPdDataSource {
 public:
  explicit ReplayPdDataSource(const std::string& dir);  // 無任何 frame → throw
  PdInput capture(const AfRequest& request) override;
  size_t frameCount() const { return frames_.size(); }

 private:
  std::vector<PdInput> frames_;
  size_t next_ = 0;
};
}  // namespace pdaf
