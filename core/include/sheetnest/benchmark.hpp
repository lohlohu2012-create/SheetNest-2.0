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
    double nfpCacheHitRate{};
    double nfpAttemptsPerPlaced{};
    double millisecondsPerPlaced{};

    std::size_t candidateChecks{};
    std::size_t collisionChecks{};
    std::size_t nfpChecks{};
    std::size_t nfpAttempts{};
    std::size_t boundsRejections{};
    std::size_t collisionRejections{};
    std::size_t feasibleCandidates{};

    std::size_t refillMoves{};
    std::size_t exchangeAttempts{};
    std::size_t sheetsEliminated{};
    std::size_t optimizerPasses{};

    std::size_t nfpTimeouts{};
    std::size_t nfpComplexityFallbacks{};
    std::size_t nfpTimeoutFallbacks{};
    std::size_t nfpCacheHits{};
    std::size_t nfpCacheMisses{};

    std::size_t telemetryNoFeasiblePosition{};
    std::size_t telemetryTimeouts{};
    std::size_t telemetryInvalidGeometry{};
    std::size_t telemetryRepairRounds{};
    std::size_t telemetryRepairExtracted{};
    std::size_t telemetryRepairMoved{};

    std::size_t initiallyPlaced{};
    std::size_t recoveredBySmallPart{};
    std::size_t recoveredByResidualRetry{};
    std::size_t recoveredOnNewSheet{};
    std::size_t recoveredByOptimizer{};
    std::size_t recoveredByAdaptiveRepair{};
    std::size_t finallyUnplaced{};

    std::vector<std::string> placedInstanceIds;
    std::vector<std::string> skippedInstanceIds;
    std::vector<InstanceNestingTelemetry> instanceTelemetry;
};

struct BenchmarkDelta {
    double milliseconds{};
    std::ptrdiff_t sheets{};
    std::ptrdiff_t placed{};
    std::ptrdiff_t skipped{};
    double utilizationPercentagePoints{};

    std::ptrdiff_t candidateChecks{};
    std::ptrdiff_t collisionChecks{};
    std::ptrdiff_t nfpChecks{};
    std::ptrdiff_t nfpAttempts{};
    std::ptrdiff_t boundsRejections{};
    std::ptrdiff_t collisionRejections{};
    std::ptrdiff_t feasibleCandidates{};

    std::ptrdiff_t refillMoves{};
    std::ptrdiff_t exchangeAttempts{};
    std::ptrdiff_t sheetsEliminated{};
    std::ptrdiff_t optimizerPasses{};

    std::ptrdiff_t nfpTimeouts{};
    std::ptrdiff_t nfpComplexityFallbacks{};
    std::ptrdiff_t nfpTimeoutFallbacks{};
    std::ptrdiff_t nfpCacheHits{};
    std::ptrdiff_t nfpCacheMisses{};

    std::ptrdiff_t telemetryNoFeasiblePosition{};
    std::ptrdiff_t telemetryTimeouts{};
    std::ptrdiff_t telemetryInvalidGeometry{};
    std::ptrdiff_t telemetryRepairRounds{};
    std::ptrdiff_t telemetryRepairExtracted{};
    std::ptrdiff_t telemetryRepairMoved{};

    std::ptrdiff_t initiallyPlaced{};
    std::ptrdiff_t recoveredBySmallPart{};
    std::ptrdiff_t recoveredByResidualRetry{};
    std::ptrdiff_t recoveredOnNewSheet{};
    std::ptrdiff_t recoveredByOptimizer{};
    std::ptrdiff_t recoveredByAdaptiveRepair{};
    std::ptrdiff_t finallyUnplaced{};

    double elapsedImprovementPercent{};
    double sheetReductionPercent{};
    double utilizationImprovementPercentagePoints{};
    double nfpCacheHitRatePercentagePoints{};
    double nfpAttemptsPerPlacedDelta{};
    double millisecondsPerPlacedDelta{};
    bool optimizedPlacementSafetyPassed{};
};

enum class BenchmarkBottleneck {
    None,
    PlacementSafety,
    InvalidGeometry,
    NfpTimeout,
    NoFeasiblePosition,
    CandidateSearch,
    SheetCapacity,
    RecoveryPressure
};

const char* benchmarkBottleneckName(BenchmarkBottleneck bottleneck);

struct BenchmarkAnalysis {
    BenchmarkBottleneck bottleneck{BenchmarkBottleneck::None};
    std::string bottleneckName;
    std::string summary;
    bool placementSafetyPassed{};
    bool lossFree{};
    bool timeImproved{};
    bool sheetsReduced{};
    bool utilizationImproved{};
    bool nfpLoadReduced{};
};

struct BenchmarkResult {
    BenchmarkCase baseline;
    BenchmarkCase optimized;
    BenchmarkDelta delta;
    BenchmarkAnalysis analysis;
};

BenchmarkAnalysis analyzeBenchmark(const BenchmarkResult& result);

BenchmarkResult benchmarkNest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& optimizedOptions
);

} // namespace sheetnest
