add_library(sheetnest_core src/geometry.cpp src/nesting.cpp src/cutting.cpp src/cutting_path.cpp src/dxf.cpp src/dxf_model.cpp)
target_include_directories(sheetnest_core PUBLIC include)
