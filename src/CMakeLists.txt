#设置 BINARY 为项目名IndexProject
set(BINARY ${CMAKE_PROJECT_NAME})

file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true *.h *.cpp)
set(SOURCES ${SOURCES})


add_executable(${BINARY}_run ${SOURCES})
# # 为了让单元测试的时候src下的代码能被作为静态链接库使用
# add_library(${BINARY}_lib STATIC ${SOURCES})

target_link_libraries(${BINARY}_run PRIVATE spdlog::spdlog doctest::doctest nanobench::nanobench)
