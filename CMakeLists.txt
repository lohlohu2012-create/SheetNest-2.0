cmake_minimum_required(VERSION 3.21)
project(SheetNest2 VERSION 0.4.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

option(SHEETNEST_BUILD_GUI "Build the SheetNest Qt desktop application when Qt 6 is available" ON)

add_subdirectory(core)

include(CTest)
enable_testing()

add_executable(sheetnest_core_smoke tests/core_smoke.cpp)
target_link_libraries(sheetnest_core_smoke PRIVATE sheetnest_core)
add_test(NAME sheetnest_core_smoke COMMAND sheetnest_core_smoke)

add_executable(sheetnest_advanced_smoke tests/advanced_smoke.cpp)
target_link_libraries(sheetnest_advanced_smoke PRIVATE sheetnest_core)
add_test(NAME sheetnest_advanced_smoke COMMAND sheetnest_advanced_smoke)

if(SHEETNEST_BUILD_GUI)
    find_package(Qt6 6.8 COMPONENTS Widgets Concurrent QUIET)
    if(Qt6_FOUND)
        add_subdirectory(app)
    else()
        message(WARNING "Qt 6 Widgets/Concurrent not found; SheetNest GUI will not be built. Core remains available.")
    endif()
endif()

set(CPACK_PACKAGE_NAME "SheetNest")
set(CPACK_PACKAGE_VENDOR "SheetNest")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_GENERATOR "ZIP")
include(CPack)
