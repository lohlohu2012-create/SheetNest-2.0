cmake_minimum_required(VERSION 3.21)
project(SheetNest2 VERSION 0.3.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_subdirectory(core)

include(CTest)
enable_testing()

add_executable(sheetnest_core_smoke tests/core_smoke.cpp)
target_link_libraries(sheetnest_core_smoke PRIVATE sheetnest_core)
add_test(NAME sheetnest_core_smoke COMMAND sheetnest_core_smoke)
