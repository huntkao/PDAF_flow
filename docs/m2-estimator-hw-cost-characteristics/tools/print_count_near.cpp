// 小工具：對硬體匯出的 window_cost*.csv（每列一個 ROI：user_N,cost_0,cost_1,...）跑
// ParabolicDepthEstimator::estimateTraced()，印出 unamb / count_near，用來檢查
// docs/m2-repeat-pattern-confidence/ 推演出的「unamb 對競爭谷數量不敏感」盲點
// 在真實硬體資料上是否也存在。
//
// 跟 run_estimate.cpp 的差異：run_estimate.cpp 吃的是單條序列的 shift,cost csv；
// 這裡吃的是硬體原始匯出格式（一個檔案內每列一個 ROI），且額外印出 count_near。
//
// 建置：
//   g++ -std=c++17 -O2 -I<repo>/include -I<repo>/src \
//       print_count_near.cpp <repo>/src/algo/parabolic_depth_estimator.cpp -o print_count_near
// 執行：
//   ./print_count_near path/to/window_cost.csv [more.csv ...]
#include <algo/parabolic_depth_estimator.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace pdaf;

static const int kShiftMin = -32; // search range 65，對稱窗 -32..32

int main(int argc, char** argv)
{
  ParabolicDepthEstimator est;
  printf("%-40s %5s %10s %10s %8s\n", "file", "roi", "unamb", "count_near", "boundary");
  for (int fi = 1; fi < argc; ++fi)
  {
    std::ifstream f(argv[fi]);
    std::string line;
    int roi = 0;
    while (std::getline(f, line))
    {
      if (line.empty())
      {
        continue;
      }
      std::stringstream ss(line);
      std::string cell;
      std::getline(ss, cell, ','); // user_N label
      CostSequence cs;
      cs.shift_min = kShiftMin;
      while (std::getline(ss, cell, ','))
      {
        cs.costs.push_back(std::stof(cell));
      }
      cs.valid_samples = static_cast<int>(cs.costs.size());
      DepthEstimateTrace t = est.estimateTraced(cs);
      printf("%-40s %5d %10.4f %10d %8s\n", argv[fi], roi, t.unamb, t.count_near,
             t.boundary ? "yes" : "no");
      ++roi;
    }
  }
  return 0;
}
