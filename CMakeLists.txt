cmake_minimum_required(VERSION 3.16)
project(pdaf_flow CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(pdaf STATIC)
target_include_directories(pdaf PUBLIC include third_party src)
# 先給一個空的翻譯單元讓 static lib 可建；後續 task 逐步加檔
file(WRITE ${CMAKE_BINARY_DIR}/pdaf_dummy.cpp "namespace pdaf { int _dummy = 0; }\n")
target_sources(pdaf PRIVATE ${CMAKE_BINARY_DIR}/pdaf_dummy.cpp src/algo/parabolic_depth_estimator.cpp src/algo/sad_cost_engine.cpp src/algo/dcc_lens_mapper.cpp src/control/af_config.cpp src/control/pdaf_pipeline.cpp src/control/af_controller.cpp src/control/run_logger.cpp src/sim/sim_world.cpp src/replay/replay_pd_data_source.cpp)

add_subdirectory(apps)

enable_testing()
add_subdirectory(tests)
