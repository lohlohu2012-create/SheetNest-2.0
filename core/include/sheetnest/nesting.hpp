#pragma once
#include "geometry.hpp"
#include <string>
#include <vector>
namespace sheetnest{struct Part{std::string id;Polygon outer;};struct Instance{std::string id;Part part;};struct Sheet{double width{},height{};};struct Placement{std::string id;double x{},y{};int rotation{};};struct Result{std::vector<std::vector<Placement>> sheets;std::vector<std::string> unplaced;double utilization{};};struct Options{std::vector<int> rotations{0,90,180,270};size_t iterations{10000};double gapMm{2};};Result nest(const std::vector<Instance>&,const Sheet&,const Options&);}
