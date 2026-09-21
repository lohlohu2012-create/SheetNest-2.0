find_package(Qt6 REQUIRED COMPONENTS Widgets)
qt_add_executable(SheetNest2 WIN32 main.cpp)
target_link_libraries(SheetNest2 PRIVATE Qt6::Widgets sheetnest_core)
