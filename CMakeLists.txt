cmake_minimum_required(VERSION 3.21)
project(SheetNest2 VERSION 0.5.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SHEETNEST_BUILD_GUI "Build Qt 6 desktop GUI" ON)

add_subdirectory(core)

enable_testing()
add_executable(sheetnest_core_tests tests/core_tests.cpp)
target_link_libraries(sheetnest_core_tests PRIVATE sheetnest_core)
add_test(NAME sheetnest_core_tests COMMAND sheetnest_core_tests)

if(SHEETNEST_BUILD_GUI)
  find_package(Qt6 6.4 COMPONENTS Widgets Concurrent QUIET)
  if(Qt6_FOUND)
    add_subdirectory(app)
  else()
    message(WARNING "Qt6 Widgets/Concurrent not found: SheetNest2App will not be built. Core remains available.")
  endif()
endif()
