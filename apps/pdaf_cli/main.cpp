#include <algo/dcc_lens_mapper.h>
#include <algo/parabolic_depth_estimator.h>
#include <algo/sad_cost_engine.h>
#include <control/pdaf_pipeline.h>
#include <control/run_logger.h>
#include <pdaf/control/af_config.h>
#include <pdaf/control/af_controller.h>
#include <replay/null_lens_actuator.h>
#include <replay/replay_pd_data_source.h>
#include <sim/sim_world.h>

#include <cstring>
#include <iostream>
#include <memory>

using namespace pdaf;

namespace
{

struct Args
{
  std::string config = "config/default.json";
  std::string mode; // 空 = 用 config 的值
  std::string out;  // 空 = 用 config 的 log_dir
  int max_frames = 200;
};

Args parseArgs(int argc, char** argv)
{
  Args a;
  for (int i = 1; i < argc; ++i)
  {
    auto need = [&](const char* name) -> const char*
    {
      if (i + 1 >= argc)
      {
        throw std::runtime_error(std::string("missing value for ") + name);
      }
      return argv[++i];
    };
    if (!std::strcmp(argv[i], "--config"))
    {
      a.config = need("--config");
    }
    else if (!std::strcmp(argv[i], "--mode"))
    {
      a.mode = need("--mode");
    }
    else if (!std::strcmp(argv[i], "--out"))
    {
      a.out = need("--out");
    }
    else if (!std::strcmp(argv[i], "--max-frames"))
    {
      a.max_frames = std::atoi(need("--max-frames"));
    }
    else
    {
      throw std::runtime_error(std::string("unknown argument: ") + argv[i]);
    }
  }
  return a;
}

} // namespace

int main(int argc, char** argv)
{
  try
  {
    const Args args = parseArgs(argc, argv);
    AfConfig cfg = AfConfig::loadFromFile(args.config);
    if (!args.mode.empty())
    {
      cfg.system.mode = args.mode;
    }
    if (!args.out.empty())
    {
      cfg.system.log_dir = args.out;
    }

    // 組裝：依 mode 選 HAL 實作，注入三模塊（demo 的 DI 就在這裡）
    std::unique_ptr<SimWorld> world;
    std::unique_ptr<IPdDataSource> source;
    std::unique_ptr<ILensActuator> actuator;
    if (cfg.system.mode == "sim")
    {
      world = std::make_unique<SimWorld>(cfg.sensor, cfg.system.sim, cfg.calibration.dcc);
      source = std::make_unique<SimPdDataSource>(*world);
      actuator = std::make_unique<SimLensActuator>(*world);
    }
    else
    {
      source = std::make_unique<ReplayPdDataSource>(cfg.system.replay_dir);
      actuator = std::make_unique<NullLensActuator>(0);
    }

    auto m1 = std::make_unique<SadCostEngine>();
    m1->init(cfg.calibration.lrc, cfg.sensor.pattern,
             cfg.tuning.shift_min, cfg.tuning.shift_max);
    PdafPipeline pipeline(std::move(m1), std::make_unique<ParabolicDepthEstimator>());
    DccLensMapper mapper;
    mapper.init(cfg.calibration.dcc);
    AfController ctrl(pipeline, mapper, *actuator, cfg.tuning);
    RunLogger logger(cfg.system.log_dir);

    ctrl.trigger();
    AfRequest req{{cfg.sensor.default_roi}};
    int frames = 0;
    for (; frames < args.max_frames; ++frames)
    {
      const PdInput in = source->capture(req);
      const AfFrameLog log = ctrl.onFrame(req, in);
      logger.logFrame(log, world ? world->groundTruthDisparity() : 0.f);
      if (ctrl.state() == AfState::kFocused || ctrl.state() == AfState::kFailed)
      {
        break;
      }
    }

    const int final_step = actuator->getStatus().current_step;
    const int gt = world ? world->inFocusStep() : -1;
    logger.writeSummary(ctrl.state(), frames + 1, final_step, gt);

    std::cout << "final_state: " << toString(ctrl.state())
              << "  frames: " << frames + 1 << "  final_step: " << final_step;
    if (world)
    {
      std::cout << "  in_focus_step: " << gt;
    }
    std::cout << "\nlogs: " << cfg.system.log_dir << "/frames.csv\n";
    return ctrl.state() == AfState::kFocused ? 0 : 1;
  }
  catch (const std::exception& e)
  {
    std::cerr << "error: " << e.what() << '\n';
    return 2;
  }
}
