#include <algo/parabolic_depth_estimator.h>
#include <pdaf/types.h>

#include <nlohmann/json.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace pdaf;
namespace fs = std::filesystem;

namespace
{

struct LoadedFrame
{
  FrameMeta meta;
  std::vector<CostSequence> costs;
  std::vector<float> gt; // 可能為空
};

struct App
{
  char path_buf[512] = "";
  std::string error;
  std::vector<LoadedFrame> frames;
  int frame_idx = 0;
  int roi_idx = 0;
};

double ys_max(const std::vector<double>& v)
{
  double m = 0;
  for (double x : v)
  {
    m = x > m ? x : m;
  }
  return m * 1.05;
}

std::optional<LoadedFrame> parseFrameFile(const fs::path& file, std::string& err)
{
  std::ifstream f(file);
  if (!f)
  {
    err = "failed to open " + file.string();
    return std::nullopt;
  }
  std::stringstream ss;
  ss << f.rdbuf();
  try
  {
    auto j = nlohmann::json::parse(ss.str());
    LoadedFrame lf;
    const auto& m = j.at("meta");
    lf.meta.frame_id = m.at("frame_id").get<uint64_t>();
    lf.meta.timestamp_ms = m.at("timestamp_ms").get<double>();
    lf.meta.lens_step_at_exposure = m.at("lens_step_at_exposure").get<int>();
    for (const auto& c : j.at("hw_costs"))
    {
      lf.costs.push_back({c.at("shift_min").get<int>(), c.at("costs").get<std::vector<float>>(),
                          c.at("valid_samples").get<int>()});
    }
    if (j.contains("ground_truth_disparity"))
    {
      lf.gt = j["ground_truth_disparity"].get<std::vector<float>>();
    }
    return lf;
  }
  catch (const std::exception& e)
  {
    err = std::string("failed to parse ") + file.filename().string() + ": " + e.what();
    return std::nullopt;
  }
}

void loadPath(App& app, const std::string& path)
{
  app.frames.clear();
  app.frame_idx = 0;
  app.roi_idx = 0;
  app.error.clear();
  if (path.empty() || !fs::exists(path))
  {
    app.error = "path does not exist: " + path;
    return;
  }
  std::vector<fs::path> files;
  if (fs::is_directory(path))
  {
    for (int i = 0;; ++i)
    {
      char name[32];
      std::snprintf(name, sizeof(name), "frame_%04d.json", i);
      const fs::path p = fs::path(path) / name;
      if (!fs::exists(p))
      {
        break;
      }
      files.push_back(p);
    }
    if (files.empty())
    {
      app.error = "no frame_0000.json in directory: " + path;
      return;
    }
  }
  else
  {
    files.push_back(path);
  }
  for (const auto& file : files)
  {
    std::string err;
    if (auto lf = parseFrameFile(file, err))
    {
      app.frames.push_back(std::move(*lf));
    }
    else
    {
      app.error = err;
    }
  }
}

// 數據面板：放在可捲動 child 內，欄位被拖窄時字會換行而非被裁掉
void drawPanel(const LoadedFrame& lf, const CostSequence& c, const DepthEstimateTrace& t, const float* gt, float height)
{
  ImGui::BeginChild("panel_scroll", ImVec2(0, height), false, ImGuiWindowFlags_HorizontalScrollbar);
  ImGui::PushTextWrapPos(0.0f);
  ImGui::Text("frame_id: %llu", static_cast<unsigned long long>(lf.meta.frame_id));
  ImGui::Text("lens@exposure: %d", lf.meta.lens_step_at_exposure);
  ImGui::Text("shift: [%d, %d]  valid: %d", c.shift_min, c.shift_min + static_cast<int>(c.costs.size()) - 1, c.valid_samples);
  ImGui::Separator();
  if (t.degenerate_no_samples)
  {
    ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "degenerate: no samples -> invalid");
  }
  else if (t.degenerate_flat)
  {
    ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "degenerate: flat (no texture) -> invalid");
  }
  else
  {
    ImGui::Text("mi: %zu   cmin: %.4f   mean: %.4f", t.mi, t.cmin, t.mean);
    ImGui::Text("depth: %.3f", t.depth);
    ImGui::Text("basin: [%zu, %zu]", t.basin_lo, t.basin_hi);
    ImGui::Text("second: %s", std::isinf(t.second) ? "inf (no competing valley)" : std::to_string(t.second).c_str());
    ImGui::Text("unamb: %.3f", t.unamb);
    if (t.boundary)
    {
      ImGui::TextColored(ImVec4(1, 0.6f, 0, 1), "boundary -> no interpolation, conf x0.5");
    }
    else
    {
      ImGui::Text("c-1,c0,c+1: %.4f, %.4f, %.4f", t.c_m1, t.c_0, t.c_p1);
      ImGui::Text("sharp: %.3f   delta: %.4f", t.sharp, t.delta);
    }
    ImGui::Separator();
    ImGui::Text("disparity: %.4f", t.result.disparity);
    ImGui::Text("confidence: %.4f", t.result.confidence);
    if (gt)
    {
      ImGui::Text("ground truth: %.4f", *gt);
      ImGui::Text("error: %.4f", std::abs(t.result.disparity - *gt));
    }
  }
  ImGui::PopTextWrapPos();
  ImGui::EndChild();
}

void drawPlot(const CostSequence& c, const DepthEstimateTrace& t, const float* gt, float height)
{
  const int nn = static_cast<int>(c.costs.size());
  std::vector<double> xs(nn), ys(nn);
  for (int i = 0; i < nn; ++i)
  {
    xs[i] = c.shift_min + i;
    ys[i] = c.costs[i];
  }
  if (ImPlot::BeginPlot("cost sequence", ImVec2(-1, height)))
  {
    ImPlot::SetupAxes("shift s", "SAD cost");
    // 預設圖例貼在圖內左上角，小視窗下會蓋住曲線本體；移到圖外上緣、橫向排列
    ImPlot::SetupLegend(ImPlotLocation_North, ImPlotLegendFlags_Outside | ImPlotLegendFlags_Horizontal);
    // basin 陰影
    if (!t.degenerate_no_samples && !t.degenerate_flat)
    {
      const double bx0 = c.shift_min + static_cast<int>(t.basin_lo);
      const double bx1 = c.shift_min + static_cast<int>(t.basin_hi);
      double bxs[2] = {bx0, bx1};
      double top[2] = {ys_max(ys), ys_max(ys)};
      double bot[2] = {0, 0};
      ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.12f);
      ImPlot::PlotShaded("basin", bxs, bot, top, 2);
      ImPlot::PopStyleVar();
    }
    // cost 曲線 + 點
    ImPlot::PlotLine("cost", xs.data(), ys.data(), nn);
    ImPlot::PlotScatter("samples", xs.data(), ys.data(), nn);
    // mean 水平線
    if (!t.degenerate_no_samples)
    {
      double mean = t.mean;
      ImPlot::PlotInfLines("mean", &mean, 1, ImPlotInfLinesFlags_Horizontal);
    }
    // 內插出的拋物線：y(u) = a·u² + b·u + c_0，u 為相對 mi 的位移
    if (!t.degenerate_no_samples && !t.degenerate_flat && !t.boundary)
    {
      const double a = 0.5 * (t.c_m1 - 2.0 * t.c_0 + t.c_p1);
      const double b = 0.5 * (t.c_p1 - t.c_m1);
      const double x0 = c.shift_min + static_cast<int>(t.mi);
      constexpr int kSeg = 64;
      std::vector<double> px(kSeg + 1), py(kSeg + 1);
      for (int i = 0; i <= kSeg; ++i)
      {
        const double u = -1.5 + 3.0 * i / kSeg;
        px[i] = x0 + u;
        py[i] = a * u * u + b * u + t.c_0;
      }
      ImPlot::SetNextLineStyle(ImVec4(0.6f, 0.8f, 1.0f, 0.9f), 2.0f);
      ImPlot::PlotLine("parabola fit", px.data(), py.data(), kSeg + 1);

      // 拋物線頂點（次像素解）
      double vx = x0 + t.delta;
      double vy = a * t.delta * t.delta + b * t.delta + t.c_0;
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Diamond, 9, ImVec4(0.6f, 0.8f, 1.0f, 1.0f), 2.0f);
      ImPlot::PlotScatter("vertex", &vx, &vy, 1);
    }
    // cmin
    if (!t.degenerate_no_samples && !t.degenerate_flat)
    {
      double cx = c.shift_min + static_cast<int>(t.mi);
      double cy = t.cmin;
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 13, ImVec4(1.0f, 0.35f, 0.35f, 0.35f), 2.5f,
                                 ImVec4(1.0f, 0.35f, 0.35f, 1.0f));
      ImPlot::PlotScatter("cmin", &cx, &cy, 1);
    }
    // 次低點（有競爭谷）
    if (!std::isinf(t.second))
    {
      // 找 second 對應的 x（basin 外的最低點）
      for (int i = 0; i < nn; ++i)
      {
        if ((i < static_cast<int>(t.basin_lo) || i > static_cast<int>(t.basin_hi)) && c.costs[i] == t.second)
        {
          double sx = xs[i], sy = ys[i];
          ImPlot::PlotScatter("second", &sx, &sy, 1);
          break;
        }
      }
    }
    // disparity 實線 + 真值改用頂部三角標記與誤差橫條，避免兩條垂直線重疊難分辨
    const ImPlotRect lim = ImPlot::GetPlotLimits();
    const ImVec4 kEst(1.0f, 0.35f, 0.35f, 1.0f);
    const ImVec4 kGt(0.3f, 0.9f, 0.5f, 1.0f);
    if (t.result.valid)
    {
      double dx = t.result.disparity;
      ImPlot::SetNextLineStyle(kEst, 2.0f);
      ImPlot::PlotInfLines("disparity", &dx, 1);
      ImPlot::Annotation(dx, lim.Y.Min, kEst, ImVec2(0, -6), true, "est %.3f", dx);
    }
    if (gt)
    {
      const double g = *gt;
      const double y_top = lim.Y.Max - 0.04 * lim.Y.Size();
      ImPlot::SetNextMarkerStyle(ImPlotMarker_Down, 11, kGt, 1.0f, kGt);
      ImPlot::PlotScatter("ground truth", &g, &y_top, 1);
      ImPlot::Annotation(g, y_top, kGt, ImVec2(0, -14), true, "GT %.3f", g);

      if (t.result.valid)
      {
        const double err_x[2] = {std::min(g, static_cast<double>(t.result.disparity)),
                                 std::max(g, static_cast<double>(t.result.disparity))};
        const double err_y[2] = {y_top, y_top};
        ImPlot::SetNextLineStyle(ImVec4(1.0f, 0.85f, 0.2f, 0.9f), 3.0f);
        ImPlot::PlotLine("error", err_x, err_y, 2);
      }
    }
    ImPlot::EndPlot();
  }
}

} // namespace

int main(int argc, char** argv)
{
  App app;
  if (argc > 1)
  {
    std::snprintf(app.path_buf, sizeof(app.path_buf), "%s", argv[1]);
    loadPath(app, argv[1]);
  }

  if (!glfwInit())
  {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  GLFWwindow* win = glfwCreateWindow(1440, 860, "pdaf_cost_viz — M2 cost sequence", nullptr, nullptr);
  if (!win)
  {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  float dpi_scale = 1.0f;
  glfwGetWindowContentScale(win, &dpi_scale, nullptr);
  if (dpi_scale > 1.01f)
  {
    // UI（含 panel 固定寬度）依 dpi_scale 放大，但視窗本身用邏輯座標建立，
    // 不補回同樣的倍率會讓可用空間被放大後的 UI 擠壓——這正是「每次都要手拉視窗」的根因。
    glfwSetWindowSize(win, static_cast<int>(1440 * dpi_scale), static_cast<int>(860 * dpi_scale));
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  ImFontConfig font_cfg;
  font_cfg.SizePixels = 13.0f * dpi_scale;
  io.Fonts->AddFontDefault(&font_cfg);
  ImGui::GetStyle().ScaleAllSizes(dpi_scale);

  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 150"); // GL 3.3 core profile 需 GLSL 150（130 在 core 不合規）

  ParabolicDepthEstimator m2;

  while (!glfwWindowShouldClose(win))
  {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->WorkPos);
    ImGui::SetNextWindowSize(vp->WorkSize);
    ImGui::Begin("main", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove);

    ImGui::SetNextItemWidth(600);
    ImGui::InputText("path (file or directory)", app.path_buf, sizeof(app.path_buf));
    ImGui::SameLine();
    if (ImGui::Button("Load"))
    {
      loadPath(app, app.path_buf);
    }
    if (!app.error.empty())
    {
      ImGui::TextColored(ImVec4(1, 0.4f, 0.4f, 1), "%s", app.error.c_str());
    }

    if (!app.frames.empty())
    {
      app.frame_idx = std::min(app.frame_idx, static_cast<int>(app.frames.size()) - 1);
      ImGui::SliderInt("frame", &app.frame_idx, 0, static_cast<int>(app.frames.size()) - 1);
      const LoadedFrame& lf = app.frames[app.frame_idx];
      if (!lf.costs.empty())
      {
        app.roi_idx = std::min(app.roi_idx, static_cast<int>(lf.costs.size()) - 1);
        if (lf.costs.size() > 1)
        {
          ImGui::SliderInt("ROI", &app.roi_idx, 0, static_cast<int>(lf.costs.size()) - 1);
        }
        const CostSequence& c = lf.costs[app.roi_idx];
        const DepthEstimateTrace t = m2.estimateTraced(c);
        const float* gt = (app.roi_idx < static_cast<int>(lf.gt.size())) ? &lf.gt[app.roi_idx] : nullptr;

        // 左：數據面板；右：圖。用 Resizable table，欄寬由使用者拖曳後保留（Columns 每幀重設會彈回）
        const float body_h = ImGui::GetContentRegionAvail().y;
        if (ImGui::BeginTable("layout", 2, ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV))
        {
          ImGui::TableSetupColumn("panel", ImGuiTableColumnFlags_WidthFixed, 340.0f * dpi_scale);
          ImGui::TableSetupColumn("plot", ImGuiTableColumnFlags_WidthStretch);
          ImGui::TableNextRow();
          ImGui::TableSetColumnIndex(0);
          drawPanel(lf, c, t, gt, body_h);
          ImGui::TableSetColumnIndex(1);
          drawPlot(c, t, gt, body_h);
          ImGui::EndTable();
        }
      }
    }
    ImGui::End();

    ImGui::Render();
    int w, h;
    glfwGetFramebufferSize(win, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}
