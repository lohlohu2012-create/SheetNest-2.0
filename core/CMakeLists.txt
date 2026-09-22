add_library(sheetnest_core src/geometry.cpp src/nesting.cpp src/cutting.cpp src/cutting_path.cpp src/dxf.cpp src/dxf_model.cpp src/dxf_export.cpp src/cam_export.cpp src/nfp.cpp src/diagnostics.cpp src/benchmark.cpp src/parallel_nesting.cpp src/production_validation.cpp src/spatial_index.cpp)
target_include_directories(sheetnest_core PUBLIC include)
