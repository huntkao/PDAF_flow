#!/usr/bin/env python3
"""自製合成 cost 曲線，測試「單一次低點 unamb」在多重近乎打平競爭谷（repeat pattern
的典型症狀）下，是否會漏掉跨 frame 雜訊造成的 argmin 誤判（AF hunting）風險。

設計：固定「最接近主谷的競爭者深度比 r」不變，只改變「有幾個谷落在同一個 r 附近」
（1 / 2 / 4 個，週期 13，避免跟主谷高斯重疊、也避開邊界 index 0/64）。
因為現有 unamb 只取 outside-basin 的全域最低點，這幾組在無雜訊時的 unamb
理論上應該幾乎不變 —— 這是要驗證的第一個假設。

對每條曲線疊加高斯雜訊做 Monte Carlo 試驗（模擬 frame-to-frame 量測雜訊），
統計 argmin 落在「非主谷」的機率（hunting rate）—— 這是 confidence 真正該預測的風險，
再跟 unamb 對照，量化兩者的落差。

用法：
  python3 synth_repeat_pattern_experiment.py <輸出資料夾> [--run-estimate <run_estimate 執行檔路徑>]

  # 若提供 --run-estimate，會額外用真正的 C++ ParabolicDepthEstimator::estimateTraced()
  # （而非本檔案內的 python 移植版）對前幾條曲線做交叉驗證，確認 python 版精確對齊生產路徑。
  # 建置方式見 docs/m2-repeat-pattern-confidence/README.md。
"""
import argparse
import csv
import math
import random
import statistics
import subprocess
from pathlib import Path

N = 65
SHIFT_MIN = -32


def gaussian_valley(center, depth, sigma, xs):
    return [depth * math.exp(-((x - center) ** 2) / (2 * sigma * sigma)) for x in xs]


def build_curve(main_center, main_depth, competitor_centers, competitor_depth,
                 base=5_000_000.0, valley_sigma=2.5):
    # 不加寬包絡：每個谷都是獨立窄高斯凹陷，谷間距（13）遠大於 sigma（2.5），
    # 確保谷之間的稜脊夠高，不會被 basin 的單調延伸吞掉，各自維持獨立的局部極小值。
    xs = list(range(SHIFT_MIN, SHIFT_MIN + N))
    v = [base] * N
    for i in range(N):
        v[i] -= gaussian_valley(main_center, main_depth, valley_sigma, xs)[i]
    for c in competitor_centers:
        dip = gaussian_valley(c, competitor_depth, valley_sigma, xs)
        for i in range(N):
            v[i] -= dip[i]
    assert min(v) > 0, "cost 曲線出現負值，不符合真實 SAD cost 恆正的物理限制"
    return xs, v


def write_csv(path, xs, v):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["shift", "cost"])
        for x, c in zip(xs, v):
            w.writerow([x, c])


def basin(v, mi):
    lo = mi
    while lo > 0 and v[lo - 1] >= v[lo]:
        lo -= 1
    hi = mi
    while hi + 1 < len(v) and v[hi + 1] >= v[hi]:
        hi += 1
    return lo, hi


def estimate_py(v):
    """純 python 移植 ParabolicDepthEstimator::estimateTraced() 的 unamb/depth 計算，
    僅用於快速 Monte Carlo；公式需與 src/algo/parabolic_depth_estimator.cpp 保持一致。"""
    n = len(v)
    mean = sum(v) / n
    mi = min(range(n), key=lambda i: v[i])
    cmin = v[mi]
    depth = max(0.0, min(1.0, 1.0 - cmin / mean))
    lo, hi = basin(v, mi)
    outside = [v[i] for i in range(n) if i < lo or i > hi]
    second = min(outside) if outside else math.inf
    depth_abs = mean - cmin
    unamb = 1.0 if (math.isinf(second) or depth_abs < 1e-9) else max(0.0, min(1.0, (second - cmin) / depth_abs))
    return dict(mi=mi, cmin=cmin, mean=mean, depth=depth, unamb=unamb, lo=lo, hi=hi, second=second)


def monte_carlo_hunt_rate(v, noise_std, trials, main_idx, tol_window=3, seed=None):
    rng = random.Random(seed)
    n = len(v)
    wrong = 0
    for _ in range(trials):
        vn = [c + rng.gauss(0.0, noise_std) for c in v]
        mi = min(range(n), key=lambda i: vn[i])
        if abs(mi - main_idx) > tol_window:
            wrong += 1
    return wrong / trials


def run_cpp(run_estimate_bin, path):
    out = subprocess.run([run_estimate_bin, path], capture_output=True, text=True, check=True).stdout
    return out.strip().splitlines()[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--run-estimate", default=None, help="run_estimate 執行檔路徑（可選，做交叉驗證）")
    ap.add_argument("--trials", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    main_depth = 3_200_000.0
    r_list = [1.05, 1.15, 1.30, 1.60]
    groups = [
        ("1 competitor", [13]),
        ("2 competitors (periodic)", [-13, 13]),
        ("4 competitors (periodic)", [-26, -13, 13, 26]),
    ]

    print(f"{'r':>5s} {'group':26s} {'unamb':>8s} {'hunt_rate':>10s}")
    rows = []
    for r in r_list:
        competitor_depth = main_depth / r
        for label, centers in groups:
            xs, v = build_curve(main_center=0, main_depth=main_depth,
                                 competitor_centers=centers, competitor_depth=competitor_depth)
            slug = label.replace(" ", "_").replace("(", "").replace(")", "")
            csv_path = outdir / f"synth_{slug}_r{r}.csv"
            write_csv(csv_path, xs, v)

            py = estimate_py(v)
            noise_std = main_depth * 0.10  # 雜訊尺度：主谷深度的 10%
            hunt = monte_carlo_hunt_rate(v, noise_std, args.trials, main_idx=py["mi"], seed=args.seed)

            rows.append((r, label, py["unamb"], hunt, csv_path))
            print(f"{r:5.2f} {label:26s} {py['unamb']:8.3f} {hunt:10.3f}")

    if args.run_estimate:
        print()
        print("--- 與真實 C++ estimateTraced()（run_estimate 工具）交叉驗證（前 3 條）---")
        for _, _, _, _, path in rows[:3]:
            print(run_cpp(args.run_estimate, path))


if __name__ == "__main__":
    main()
