#pragma once
#include "geometry.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sheetnest {

struct Part {
    std::string id;
    Polygon outer;
    std::vector<Polygon> holes;
    std::string sourceId;
    std::string layer;
};

struct Instance {
    std::string id;
    Part part;
    std::string unitId;
};

struct Sheet {
    double width{};
    double height{};
    double edgeMarginMm{};
};

struct Placement {
    std::string id;
    double x{};
    double y{};
    int rotation{};
};

struct Result {
    std::vector<std::vector<Placement>> sheets;
    std::vector<std::string> unplaced;
    double utilization{};
};

struct Options {
    std::vector<int> rotations{0, 90, 180, 270};
    std::size_t iterations{24};
    double gapMm{2.0};
    std::uint32_t seed{0x534E4553u};
};

Result nest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options
);

} // namespace sheetnest
