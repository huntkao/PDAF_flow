include(FetchContent)
FetchContent_Declare(googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(pdaf_tests test_smoke.cpp test_depth_estimator.cpp test_cost_engine.cpp test_lens_mapper.cpp test_null_actuator.cpp test_af_config.cpp test_pdaf_pipeline.cpp test_af_controller.cpp test_sim_world.cpp test_replay_source.cpp test_run_logger.cpp test_closed_loop.cpp test_cost_dump.cpp)
target_link_libraries(pdaf_tests PRIVATE pdaf GTest::gtest_main)
target_compile_definitions(pdaf_tests PRIVATE PDAF_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(pdaf_tests)
