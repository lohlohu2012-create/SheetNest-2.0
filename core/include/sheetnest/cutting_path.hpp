#pragma once
#include "geometry.hpp"
#include "cutting.hpp"
#include <vector>
namespace sheetnest {
enum class CutType { Pierce, Cut, Rapid };
struct ToolMove { CutType type; Point from{},to{}; double lengthMm{}; double speedMMin{}; };
struct CuttingPath { std::vector<ToolMove> moves; double totalCutLengthMm{}; double totalRapidLengthMm{}; int pierces{}; };
struct PathOptions { double rapidSpeedMMin{120}; double pierceSeconds{0.25}; double cornerSlowdown{0.85}; };
CuttingPath planCuttingPath(const std::vector<Polygon>& contours,const CuttingParameters&,const PathOptions&);
CuttingEstimate estimateCuttingPath(const CuttingPath&,const CuttingParameters&,const PathOptions&);
}
