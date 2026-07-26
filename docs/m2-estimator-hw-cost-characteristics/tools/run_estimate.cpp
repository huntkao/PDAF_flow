// 小工具：對一批「單條 cost 序列」csv（欄位 shift,cost）跑 ParabolicDepthEstimator::estimateTraced()，
// 印出 disparity / confidence 與中間量（mi / boundary / depth / unamb / sharp），方便對照真實硬體資料。
//
// 建置：
//   g++ -std=c++17 -O2 -I<repo>/include -I<repo>/src \
//       run_estimate.cpp <repo>/src/algo/parabolic_depth_estimator.cpp -o run_estimate
// 執行：
//   ./run_estimate path/to/user_0.csv path/to/user_1.csv ...
//
// 輸入 csv 格式（第一列為 header，之後每列一個取樣點）：
//   shift,cost
//   -32,4032610
//   -31,4979135
//   ...
#include <algo/parabolic_depth_estimator.h>

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace pdaf;

static CostSequence loadCsv(const std::string& path)
{
  CostSequence cs;
  std::ifstream f(path);
  std::string line;
  std::getline(f, line); // header
  bool first = true;
  while (std::getline(f, line))
  {
    if (line.empty())
    {
      continue;
    }
    std::stringstream ss(line);
    std::string shift_str, cost_str;
    std::getline(ss, shift_str, ',');
    std::getline(ss, cost_str, ',');
    if (first)
    {
      cs.shift_min = std::stoi(shift_str);
      first = false;
    }
    cs.costs.push_back(std::stof(cost_str));
  }
  cs.valid_samples = static_cast<int>(cs.costs.size());
  return cs;
}

int main(int argc, char** argv)
{
  ParabolicDepthEstimator est;
  printf("%-45s %10s %10s %8s %8s %8s %8s %8s\n", "file", "disparity", "confidence", "mi", "boundary", "depth", "unamb", "sharp");
  for (int i = 1; i < argc; ++i)
  {
    std::string path = argv[i];
    CostSequence cs = loadCsv(path);
    DepthEstimateTrace t = est.estimateTraced(cs);
    printf("%-45s %10.4f %10.4f %8zu %8s %8.4f %8.4f %8.4f\n",
           path.c_str(), t.result.disparity, t.result.confidence, t.mi,
           t.boundary ? "yes" : "no", t.depth, t.unamb, t.sharp);
  }
  return 0;
}
