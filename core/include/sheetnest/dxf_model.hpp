#pragma once

#include "dxf.hpp"
#include "nesting.hpp"

#include <cstddef>
#include <vector>

namespace sheetnest {

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
