#pragma once

#include "nesting.hpp"
#include <string>

namespace sheetnest {
struct RuntimeTuningReport {
    Options options{};
    bool tuned{};
    std::string profile;
    std::string reason;
    std::size_t totalVertices{};
    std::size_t maxPartVertices{};
    std::size_t instanceCount{};
};
RuntimeTuningReport tuneNestingOptions(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& baseOptions
);
} // namespace sheetnest
