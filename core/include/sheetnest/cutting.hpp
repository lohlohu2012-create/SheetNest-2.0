#pragma once
#include <map>
#include <string>
#include <vector>
namespace sheetnest {
enum class Material { CarbonSteel, StainlessSteel, Aluminum, Brass, MildSteel };
struct CuttingParameters { Material material{}; double thicknessMm{}; double speedMMin{}; std::string assistGas; };
struct CutContour { std::string id; double lengthMm{}; bool closed{true}; };
struct CuttingEstimate { double contourLengthMm{}; double cuttingMinutes{}; double piercingMinutes{}; double rapidMinutes{}; double totalMinutes{}; int pierces{}; CuttingParameters parameters; };
CuttingParameters bodor3kWParameters(Material material,double thicknessMm);
CuttingEstimate estimateCutting(const std::vector<CutContour>& contours,const CuttingParameters&,int pierces=0,double rapidLengthMm=0,double pierceSeconds=0.25);
}
