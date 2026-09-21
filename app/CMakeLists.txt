set(CMAKE_AUTOMOC ON)

add_executable(SheetNest
    main.cpp
    mainwindow.cpp
    mainwindow.hpp
    nestview.cpp
    nestview.hpp
)

target_link_libraries(SheetNest
    PRIVATE
        sheetnest_core
        Qt6::Widgets
        Qt6::Concurrent
)

target_include_directories(SheetNest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(WIN32)
    set_target_properties(SheetNest PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

install(TARGETS SheetNest
    RUNTIME DESTINATION .
    BUNDLE DESTINATION .
)

qt_generate_deploy_app_script(
    TARGET SheetNest
    OUTPUT_SCRIPT sheetnest_deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${sheetnest_deploy_script})

install(FILES ${CMAKE_SOURCE_DIR}/README.md DESTINATION .)
