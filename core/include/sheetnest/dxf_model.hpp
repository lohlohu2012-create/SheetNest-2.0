#pragma once

#include "dxf.hpp"
#include "nesting.hpp"

#include <cstddef>
#include <vector>

namespace sheetnest {

struct DxfPreflightReport {
    bool valid{true};
    std::size_t validParts{};
    std::size_t emptyParts{};
    std::size_t invalidParts{};
    double minArea{};
    double maxArea{};
    std::vector<std::string> issues;
};

DxfPreflightReport preflightDxf(
    const DxfDocument& document
);

std::vector<Part> partsFromDxf(
    const DxfDocument& document
);

std::vector<Instance> instancesFromDxf(
    const DxfDocument& document,
    std::size_t quantityPerPart = 1
);

std::vector<Instance> instancesFromDxf(
    const DxfDocument& document,
    const std::vector<std::size_t>& quantities
);

} // namespace sheetnest
