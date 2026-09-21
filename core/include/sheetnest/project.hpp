#pragma once
#include "cutting_path.hpp"
#include "nesting.hpp"
#include <string>
namespace sheetnest {
struct ProjectSettings { Sheet sheet; Material material{Material::CarbonSteel}; double thicknessMm{1}; double gapMm{2}; double edgeMarginMm{5}; int laserPowerW{3000}; CuttingParameters cutting; };
struct ProjectEstimate { CuttingEstimate total; std::vector<CuttingEstimate> perSheet; };
}
