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
    std::size_t candidateChecks{};
    std::size_t collisionChecks{};
    std::size_t nfpChecks{};
    std::size_t refillMoves{};
    std::size_t exchangeAttempts{};
    std::size_t sheetsEliminated{};
    std::size_t optimizerPasses{};
    std::size_t nfpTimeouts{};
    std::size_t nfpComplexityFallbacks{};
    std::size_t nfpCacheHits{};
    std::size_t nfpCacheMisses{};
    std::vector<std::string> placedInstanceIds;
    std::vector<std::string> skippedInstanceIds;
    std::vector<InstanceNestingTelemetry> instanceTelemetry;
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
