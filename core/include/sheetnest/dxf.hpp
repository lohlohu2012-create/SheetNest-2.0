#pragma once
#include "geometry.hpp"
#include <string>
#include <vector>
namespace sheetnest {
struct DxfContour { Polygon outer; std::vector<Polygon> holes; std::string sourceId; };
struct DxfDocument { std::vector<DxfContour> contours; std::string warnings; };
DxfDocument importDxf(const std::string& text, double arcToleranceMm=0.25);
}
