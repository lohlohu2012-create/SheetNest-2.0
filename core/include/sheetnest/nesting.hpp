#pragma once
#include "geometry.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sheetnest {
struct Part { std::string id; Shape shape; };
struct Instance { std::string id; Part part; };
struct Sheet { double width{}, height{}, marginMm{}; };
struct Placement { std::string id; double x{}, y{}; int rotation{}; };
struct Result {
  std::vector<std::vector<Placement>> sheets;
  std::vector<std::string> unplaced;
  double utilization{};
  double usedAreaMm2{};
  std::size_t iterations{};
};
struct Options {
  std::vector<int> rotations{0,90,180,270};
  std::size_t iterations{64};
  double gapMm{2};
  double candidateGridMm{5};
  std::uint64_t seed{0};
};
Result nest(const std::vector<Instance>&, const Sheet&, const Options&);
}
