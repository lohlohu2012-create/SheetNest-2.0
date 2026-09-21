#pragma once

#include "nesting.hpp"

#include <cstddef>
#include <vector>
#include <string>

namespace sheetnest {

struct BenchmarkCase {
    std::string name;
    double milliseconds{};
    std::size_t sheets{};
    std::size_t placed{};
    std::size_t skipped{};
    double utilization{};
};

struct BenchmarkResult {
    BenchmarkCase baseline;
    BenchmarkCase optimized;
};

BenchmarkResult benchmarkNest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& optimizedOptions
);

} // namespace sheetnest
