#pragma once
#include "geometry.hpp"
#include <string>
#include <vector>
namespace sheetnest{struct DxfContour{Polygon outer;std::vector<Polygon>holes;std::string sourceId;};struct DxfDocument{std::vector<DxfContour>contours;};DxfDocument importDxf(const std::string&,double arcToleranceMm=.25);}
