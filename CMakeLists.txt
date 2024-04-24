cmake_minimum_required(VERSION 3.10...3.29)
project(shorthpath VERSION 0.1.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 20)
include_directories(src)
include_directories(lib/spdlog/include)


set(BENCHMARK_ENABLE_GTEST_TESTS OFF) 
add_subdirectory(src)
add_subdirectory(test)
add_subdirectory(benchmark)
add_subdirectory(lib/googletest)
add_subdirectory(lib/benchmark)
add_subdirectory(lib/spdlog)