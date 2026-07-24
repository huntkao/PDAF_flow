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
    err = "無法開啟 " + file.string();
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
    err = std::string("解析失敗 ") + file.filename().string() + ": " + e.what();
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
    app.error = "路徑不存在：" + path;
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
      app.error = "目錄下無 frame_0000.json：" + path;
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

void drawPlot(const CostSequence& c, const DepthEstimateTrace& t, const float* gt)
{
  const int nn = static_cast<int>(c.costs.size());
  std::vector<double> xs(nn), ys(nn);
  for (int i = 0; i < nn; ++i)
  {
    xs[i] = c.shift_min + i;
    ys[i] = c.costs[i];
  }
  if (ImPlot::BeginPlot("cost sequence", ImVec2(-1, -1)))
  {
    ImPlot::SetupAxes("shift s", "SAD cost");
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
    // cmin
    if (!t.degenerate_no_samples && !t.degenerate_flat)
    {
      double cx = c.shift_min + static_cast<int>(t.mi);
      double cy = t.cmin;
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
    // disparity 垂直線
    if (t.result.valid)
    {
      double dx = t.result.disparity;
      ImPlot::PlotInfLines("disparity", &dx, 1);
    }
    // 真值
    if (gt)
    {
      double g = *gt;
      ImPlot::PlotInfLines("ground truth", &g, 1);
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
  GLFWwindow* win = glfwCreateWindow(1100, 720, "pdaf_cost_viz — M2 cost sequence", nullptr, nullptr);
  if (!win)
  {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 130");

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
    ImGui::InputText("路徑（檔案或目錄）", app.path_buf, sizeof(app.path_buf));
    ImGui::SameLine();
    if (ImGui::Button("載入"))
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

        // 左：數據面板；右：圖
        ImGui::Columns(2, nullptr, true);
        ImGui::SetColumnWidth(0, 340);
        ImGui::Text("frame_id: %llu", static_cast<unsigned long long>(lf.meta.frame_id));
        ImGui::Text("lens@exposure: %d", lf.meta.lens_step_at_exposure);
        ImGui::Text("shift: [%d, %d]  valid: %d", c.shift_min,
                    c.shift_min + static_cast<int>(c.costs.size()) - 1, c.valid_samples);
        ImGui::Separator();
        if (t.degenerate_no_samples)
        {
          ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "degenerate: no samples → invalid");
        }
        else if (t.degenerate_flat)
        {
          ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "degenerate: flat (無紋理) → invalid");
        }
        else
        {
          ImGui::Text("mi: %zu   cmin: %.4f   mean: %.4f", t.mi, t.cmin, t.mean);
          ImGui::Text("depth: %.3f", t.depth);
          ImGui::Text("basin: [%zu, %zu]", t.basin_lo, t.basin_hi);
          ImGui::Text("second: %s", std::isinf(t.second) ? "inf (無競爭谷)" : std::to_string(t.second).c_str());
          ImGui::Text("unamb: %.3f", t.unamb);
          if (t.boundary)
          {
            ImGui::TextColored(ImVec4(1, 0.6f, 0, 1), "boundary → 不內插、conf×0.5");
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
        ImGui::NextColumn();
        drawPlot(c, t, gt);
        ImGui::Columns(1);
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
