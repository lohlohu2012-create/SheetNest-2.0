#pragma once
#include "geometry.hpp"
#include "cutting.hpp"
#include <vector>
namespace sheetnest{enum class CutType{Pierce,Cut,Rapid};struct ToolMove{CutType type;Point from{},to{};double lengthMm{},speedMMin{};};struct CuttingPath{std::vector<ToolMove> moves;double totalCutLengthMm{},totalRapidLengthMm{};int pierces{};};struct PathOptions{double rapidSpeedMMin{120},pierceSeconds{.25};bool innerContoursFirst{true};};CuttingPath planCuttingPath(const std::vector<Polygon>&,const CuttingParameters&,const PathOptions&);CuttingEstimate estimateCuttingPath(const CuttingPath&,const CuttingParameters&,const PathOptions&);}
