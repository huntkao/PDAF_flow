# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

PDAF_flow is a C++17 demo framework for phase-detection autofocus (PDAF) flow development. It is designed from a real hardware-application viewpoint: it owns the whole AF system's configuration and wires three algorithm modules into a closed loop that drives a lens to focus. There is no real sensor/VCM — an optical simulator closes the loop for demo and testing, and a replay source feeds recorded dumps for offline analysis.

The full design rationale is in `docs/superpowers/specs/2026-07-22-pdaf-flow-design.md`; the task-by-task build plan (with the exact reference code for every module) is in `docs/superpowers/plans/2026-07-22-pdaf-flow-framework.md`. Read the spec before making architectural changes. The Qualcomm AF stack this layering is modeled on, and the verified public sources for it, are documented in `docs/ref/qualcomm/README.md` (the comparison diagram is a conceptual synthesis, not an official Qualcomm document — the patents there are the substantive sources).

## Build, test, run

# The build is driven by CMakePresets.json (Ninja generator, binaryDir = build/).
# Always use the presets — a plain `cmake -B build` can pick a different default
# generator and collide with the preset's build/ cache (generator mismatch).
cmake --preset default                            # configure (first time or after CMakeLists edits)
cmake --build --preset default                    # build (incremental)
ctest --preset default                            # run the whole suite (outputs on failure)
#   presets: default (Debug) · release (Release). Requires cmake >= 3.21 + ninja.

# run a single test suite or case (single test binary, GoogleTest filter):
./build/tests/pdaf_tests --gtest_filter='ClosedLoop.*'
./build/tests/pdaf_tests --gtest_filter='AfController.LensCommandUsesExposureStepNotActuatorPosition'

# run the demo CLI (sim mode; converges to FOCUSED and writes logs):
./build/apps/pdaf_cli --config config/default.json --out out
#   config/{default,near,far}.json are three convergence scenarios (mid / near / far distance)
#   --mode replay reads frame dumps from system.replay_dir instead of the simulator
#   outputs: <out>/frames.csv (per-frame state/disparity/confidence/lens_step) + <out>/summary.txt
```

There is no separate linter. All tests compile into one binary, `pdaf_tests` — new test files are added to the `add_executable(pdaf_tests ...)` list in `tests/CMakeLists.txt`, not as new targets. GoogleTest (v1.14.0) is fetched via FetchContent; nlohmann/json (v3.11.3) is vendored at `third_party/nlohmann/json.hpp`. No OpenCV — all image math is hand-written in the simulator.

## Architecture: four layers

The core builds into `libpdaf` (static); `pdaf_cli` and `pdaf_tests` link it. Everything is in `namespace pdaf`. Public interfaces live in `include/pdaf/`; reference implementations of those interfaces live in `src/` (and `src/` is on the library's public include path, so tests include impl headers like `<algo/sad_cost_engine.h>`).

- **HAL** (`include/pdaf/hal/`): `IPdDataSource::capture(AfRequest) → PdInput` and `ILensActuator`. The simulator (`src/sim/`) and replay (`src/replay/`) are just different implementations. Swapping in real hardware means writing one more implementation — nothing above the HAL changes.
- **Algorithm modules** (`include/pdaf/algo/`, impls in `src/algo/`): **M1** `IPdCostEngine`/`SadCostEngine` (LRC gain correction + SAD matching cost per ROI), **M2** `IDepthEstimator`/`ParabolicDepthEstimator` (cost-minimum + parabolic sub-pixel → disparity + confidence), **M3** `ILensMapper`/`DccLensMapper` (DCC interpolation → target VCM step). Each is a pure interface with one reference impl; replace an impl without touching consumers.
- **Control** (`include/pdaf/control/`, impls in `src/control/`): `PdafPipeline` wraps M1+M2 behind `IFocusEstimator` (the HAF-style arbitration seam); `AfController` is the single-shot AF state machine; `AfConfig` loads JSON. `AfController` depends **only** on `IFocusEstimator`, `ILensMapper`, `ILensActuator` — it never sees a concrete type. Assembly (choosing sim vs replay, injecting the three modules) happens once, in `apps/pdaf_cli/main.cpp`.
- **App** (`apps/pdaf_cli/`): reads config, assembles the pipeline per mode, runs the per-frame loop to FOCUSED/FAILED, writes logs via `RunLogger`.

Data flow per frame: `PdInput` → `PdafPipeline` (M1 then M2, or HW-cost path bypassing M1) → `DepthEstimate` → `AfController` decides → `DccLensMapper` → `LensCommand` → actuator. `AfController::onFrame` drives one sensor frame and returns an `AfFrameLog`; the state machine is `kIdle→kMeasuring→kMoving→kSettling→kVerifying→kFocused/kFailed`.

## Invariants you must not break

These are load-bearing and span multiple files — the reason a change can pass locally but be wrong:

- **M3's base position is the exposure-time lens step, not the actuator's current position.** `AfController` passes `input.meta.lens_step_at_exposure` to `ILensMapper::toLensCommand` — never `actuator.getStatus().current_step`. This models sensor pipeline latency and is the precondition for future CAF. `AfController.LensCommandUsesExposureStepNotActuatorPosition` guards it; if you refactor the controller, keep that test meaningful (it must fail if the base is switched to the actuator position).
- **The simulator and M3 share the same `dccInterp` free function** (`include/pdaf/algo/lens_mapper.h`). The sim derives its ground-truth disparity through `dccInterp`, so the closed loop is self-consistent. Don't fork a second copy of that math into the simulator.
- **Simulator texture frequencies must stay below π/(max_shift) ≈ 0.196 rad/sample** (for the ±16 shift window). `SimWorld::texture` uses `{0.07, 0.13, 0.19}`. A higher frequency whose period approaches the search-window width produces a false SAD cost minimum (aliasing) and breaks disparity measurement. If you widen the shift search window or change the texture, re-check this inequality.
- **Runtime algorithm/control code never throws for bad data** — it degrades (invalid `DepthEstimate` with `confidence=0`, or a state-machine retry/fail transition). Only `AfConfig` loading fails fast, throwing `std::runtime_error` with the offending field path.

## Convention notes

- **Code style is Allman with mandatory braces**, enforced by `.clang-format` (based on LLVM, 2-space indent, `ColumnLimit: 0`). Every `{` goes on its own line, and single-statement `if`/`for`/etc. get explicit braces with the body on a separate line (`InsertBraces: true`). Pointer/reference `&`/`*` stick to the type (`const T& x`). Run `clang-format -i` on any file you touch (the pip-installed binary lives at `~/.local/bin/clang-format`); don't hand-format against this.
- The closed-loop integration test (`tests/test_closed_loop.cpp`) is the framework's living spec: it runs the real simulator + real M1/M2/M3 end-to-end and asserts convergence to FOCUSED within tolerance across mid/near/far scenarios. If a change makes it fail, that is a real integration regression — do not loosen the assertions to make it pass.
- `PdInput` carries either raw `PdFrame` samples or pre-computed `hw_costs` (ISP hardware path); `PdafPipeline` runs M1 only on the raw path. `IFocusEstimator`'s seam is real for alternative PD estimators, but a genuinely different modality (contrast/ToF) would need `PdInput` extended with a new data arm.
