#pragma once
#include "geometry.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sheetnest {
struct Part { std::string id; Shape shape; };
struct Instance { std::string id; Part part; std::string unitId{}; };
struct Sheet { double width{}, height{}, marginMm{}; };
struct Placement { std::string id; double x{}, y{}; int rotation{}; std::string unitId{}; };
struct PlacementDiagnostic {
  std::string instanceId;
  std::string unitId;
  std::string stage;
  std::string message;
};
struct Result {
  std::vector<std::vector<Placement>> sheets;
  std::vector<std::string> unplaced;
  std::vector<PlacementDiagnostic> diagnostics;
  double utilization{};
  double usedAreaMm2{};
  double sheetWidthMm{};
  double sheetHeightMm{};
  std::size_t iterations{};
  bool complete() const { return unplaced.empty(); }
};
struct Options {
  std::vector<int> rotations{0,90,180,270};
  std::size_t iterations{64};
  double gapMm{2};
  double candidateGridMm{5};
  std::uint64_t seed{0};
  std::size_t parallelism{0}; // 0 = auto-detect CPU parallelism
  std::size_t sheetReductionPasses{4}; // attempts per target sheet count
};
Result nest(const std::vector<Instance>&, const Sheet&, const Options&);
}
