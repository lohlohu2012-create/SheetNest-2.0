set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

add_executable(SheetNest2App WIN32
  src/main.cpp
  src/mainwindow.cpp
  src/nesting_view.cpp
  resources/app.qrc
)

target_link_libraries(SheetNest2App PRIVATE
  sheetnest_core
  Qt6::Widgets
  Qt6::Concurrent
)

target_include_directories(SheetNest2App PRIVATE src)

install(TARGETS SheetNest2App RUNTIME DESTINATION .)
