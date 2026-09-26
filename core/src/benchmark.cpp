#include "sheetnest/benchmark.hpp"
#include "sheetnest/nfp.hpp"

#include <chrono>
#include <unordered_set>

namespace sheetnest {
namespace {

BenchmarkCase runCase(
    const std::string& name,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    Options options
) {
    // Every case starts from an equivalent cold cache and an independent
    // control object. A caller's cancelled/expired control must never make
    // one side of the benchmark silently terminate earlier than the other.
    nfp::clearCache();
    options.control.reset();

    const auto started = std::chrono::steady_clock::now();
    const auto result = nest(instances, sheet, options);
    const auto finished = std::chrono::steady_clock::now();

    BenchmarkCase benchmark;
    benchmark.name = name;
    benchmark.milliseconds =
        std::chrono::duration<double, std::milli>(
            finished - started
        ).count();

    benchmark.sheets = result.sheets.size();
    benchmark.skipped = result.unplaced.size();
    benchmark.placed =
        instances.size() >= benchmark.skipped
            ? instances.size() - benchmark.skipped
            : 0;
    benchmark.utilization = result.utilization;

    benchmark.candidateChecks = result.stats.candidateChecks;
    benchmark.collisionChecks = result.stats.collisionChecks;
    benchmark.nfpChecks = result.stats.nfpChecks;
    benchmark.nfpAttempts = result.stats.nfpAttempts;
    benchmark.boundsRejections = result.stats.boundsRejections;
    benchmark.collisionRejections = result.stats.collisionRejections;
    benchmark.feasibleCandidates = result.stats.feasibleCandidates;

    benchmark.refillMoves = result.stats.refillMoves;
    benchmark.exchangeAttempts = result.stats.exchangeAttempts;
    benchmark.sheetsEliminated = result.stats.sheetsEliminated;
    benchmark.optimizerPasses = result.stats.optimizerPasses;

    benchmark.nfpTimeouts = result.stats.nfpTimeouts;
    benchmark.nfpComplexityFallbacks =
        result.stats.nfpComplexityFallbacks;
    benchmark.nfpTimeoutFallbacks =
        result.stats.nfpTimeoutFallbacks;
    benchmark.nfpCacheHits = result.stats.nfpCacheHits;
    benchmark.nfpCacheMisses = result.stats.nfpCacheMisses;

    std::unordered_set<std::string> skipped(
        result.unplaced.begin(),
        result.unplaced.end()
    );

    benchmark.skippedInstanceIds = result.unplaced;
    benchmark.instanceTelemetry = result.instanceTelemetry;

    benchmark.placedInstanceIds.reserve(benchmark.placed);
    for (const auto& instance : instances) {
        if (!skipped.contains(instance.id)) {
            benchmark.placedInstanceIds.push_back(instance.id);
        }
    }

    // Telemetry is authoritative for diagnosing why a specific instance was
    // lost. Aggregate it separately from the low-level engine counters so
    // the benchmark can answer both "how much work was done?" and
    // "what actually failed?".
    for (const auto& telemetry : result.instanceTelemetry) {
        switch (telemetry.reason) {
            case NestingFailureReason::NoFeasiblePosition:
                ++benchmark.telemetryNoFeasiblePosition;
                break;
            case NestingFailureReason::Timeout:
                ++benchmark.telemetryTimeouts;
                break;
            case NestingFailureReason::InvalidGeometry:
                ++benchmark.telemetryInvalidGeometry;
                break;
            case NestingFailureReason::None:
            case NestingFailureReason::RepairExhausted:
            case NestingFailureReason::Cancelled:
                break;
        }

        benchmark.telemetryRepairRounds += telemetry.repairRounds;
        if (telemetry.repairExtracted) {
            ++benchmark.telemetryRepairExtracted;
        }
        if (telemetry.repairMoved) {
            ++benchmark.telemetryRepairMoved;
        }
    }

    return benchmark;
}

BenchmarkDelta makeDelta(
    const BenchmarkCase& baseline,
    const BenchmarkCase& optimized
) {
    BenchmarkDelta delta;

    delta.milliseconds =
        optimized.milliseconds - baseline.milliseconds;
    delta.sheets =
        static_cast<std::ptrdiff_t>(optimized.sheets) -
        static_cast<std::ptrdiff_t>(baseline.sheets);
    delta.placed =
        static_cast<std::ptrdiff_t>(optimized.placed) -
        static_cast<std::ptrdiff_t>(baseline.placed);
    delta.skipped =
        static_cast<std::ptrdiff_t>(optimized.skipped) -
        static_cast<std::ptrdiff_t>(baseline.skipped);
    delta.utilizationPercentagePoints =
        (optimized.utilization - baseline.utilization) * 100.0;

    delta.candidateChecks =
        static_cast<std::ptrdiff_t>(optimized.candidateChecks) -
        static_cast<std::ptrdiff_t>(baseline.candidateChecks);
    delta.collisionChecks =
        static_cast<std::ptrdiff_t>(optimized.collisionChecks) -
        static_cast<std::ptrdiff_t>(baseline.collisionChecks);
    delta.nfpChecks =
        static_cast<std::ptrdiff_t>(optimized.nfpChecks) -
        static_cast<std::ptrdiff_t>(baseline.nfpChecks);
    delta.nfpAttempts =
        static_cast<std::ptrdiff_t>(optimized.nfpAttempts) -
        static_cast<std::ptrdiff_t>(baseline.nfpAttempts);
    delta.boundsRejections =
        static_cast<std::ptrdiff_t>(optimized.boundsRejections) -
        static_cast<std::ptrdiff_t>(baseline.boundsRejections);
    delta.collisionRejections =
        static_cast<std::ptrdiff_t>(optimized.collisionRejections) -
        static_cast<std::ptrdiff_t>(baseline.collisionRejections);
    delta.feasibleCandidates =
        static_cast<std::ptrdiff_t>(optimized.feasibleCandidates) -
        static_cast<std::ptrdiff_t>(baseline.feasibleCandidates);

    delta.refillMoves =
        static_cast<std::ptrdiff_t>(optimized.refillMoves) -
        static_cast<std::ptrdiff_t>(baseline.refillMoves);
    delta.exchangeAttempts =
        static_cast<std::ptrdiff_t>(optimized.exchangeAttempts) -
        static_cast<std::ptrdiff_t>(baseline.exchangeAttempts);
    delta.sheetsEliminated =
        static_cast<std::ptrdiff_t>(optimized.sheetsEliminated) -
        static_cast<std::ptrdiff_t>(baseline.sheetsEliminated);
    delta.optimizerPasses =
        static_cast<std::ptrdiff_t>(optimized.optimizerPasses) -
        static_cast<std::ptrdiff_t>(baseline.optimizerPasses);

    delta.nfpTimeouts =
        static_cast<std::ptrdiff_t>(optimized.nfpTimeouts) -
        static_cast<std::ptrdiff_t>(baseline.nfpTimeouts);
    delta.nfpComplexityFallbacks =
        static_cast<std::ptrdiff_t>(optimized.nfpComplexityFallbacks) -
        static_cast<std::ptrdiff_t>(baseline.nfpComplexityFallbacks);
    delta.nfpTimeoutFallbacks =
        static_cast<std::ptrdiff_t>(optimized.nfpTimeoutFallbacks) -
        static_cast<std::ptrdiff_t>(baseline.nfpTimeoutFallbacks);
    delta.nfpCacheHits =
        static_cast<std::ptrdiff_t>(optimized.nfpCacheHits) -
        static_cast<std::ptrdiff_t>(baseline.nfpCacheHits);
    delta.nfpCacheMisses =
        static_cast<std::ptrdiff_t>(optimized.nfpCacheMisses) -
        static_cast<std::ptrdiff_t>(baseline.nfpCacheMisses);

    delta.telemetryNoFeasiblePosition =
        static_cast<std::ptrdiff_t>(
            optimized.telemetryNoFeasiblePosition
        ) -
        static_cast<std::ptrdiff_t>(
            baseline.telemetryNoFeasiblePosition
        );
    delta.telemetryTimeouts =
        static_cast<std::ptrdiff_t>(optimized.telemetryTimeouts) -
        static_cast<std::ptrdiff_t>(baseline.telemetryTimeouts);
    delta.telemetryInvalidGeometry =
        static_cast<std::ptrdiff_t>(
            optimized.telemetryInvalidGeometry
        ) -
        static_cast<std::ptrdiff_t>(
            baseline.telemetryInvalidGeometry
        );
    delta.telemetryRepairRounds =
        static_cast<std::ptrdiff_t>(optimized.telemetryRepairRounds) -
        static_cast<std::ptrdiff_t>(baseline.telemetryRepairRounds);
    delta.telemetryRepairExtracted =
        static_cast<std::ptrdiff_t>(
            optimized.telemetryRepairExtracted
        ) -
        static_cast<std::ptrdiff_t>(
            baseline.telemetryRepairExtracted
        );
    delta.telemetryRepairMoved =
        static_cast<std::ptrdiff_t>(
            optimized.telemetryRepairMoved
        ) -
        static_cast<std::ptrdiff_t>(
            baseline.telemetryRepairMoved
        );

    delta.initiallyPlaced =
        static_cast<std::ptrdiff_t>(optimized.initiallyPlaced) -
        static_cast<std::ptrdiff_t>(baseline.initiallyPlaced);
    delta.recoveredBySmallPart =
        static_cast<std::ptrdiff_t>(optimized.recoveredBySmallPart) -
        static_cast<std::ptrdiff_t>(baseline.recoveredBySmallPart);
    delta.recoveredByResidualRetry =
        static_cast<std::ptrdiff_t>(optimized.recoveredByResidualRetry) -
        static_cast<std::ptrdiff_t>(baseline.recoveredByResidualRetry);
    delta.recoveredOnNewSheet =
        static_cast<std::ptrdiff_t>(optimized.recoveredOnNewSheet) -
        static_cast<std::ptrdiff_t>(baseline.recoveredOnNewSheet);
    delta.recoveredByOptimizer =
        static_cast<std::ptrdiff_t>(optimized.recoveredByOptimizer) -
        static_cast<std::ptrdiff_t>(baseline.recoveredByOptimizer);
    delta.recoveredByAdaptiveRepair =
        static_cast<std::ptrdiff_t>(optimized.recoveredByAdaptiveRepair) -
        static_cast<std::ptrdiff_t>(baseline.recoveredByAdaptiveRepair);
    delta.finallyUnplaced =
        static_cast<std::ptrdiff_t>(optimized.finallyUnplaced) -
        static_cast<std::ptrdiff_t>(baseline.finallyUnplaced);

    if (baseline.milliseconds > 0.0) {
        delta.elapsedImprovementPercent =
            (baseline.milliseconds - optimized.milliseconds) /
            baseline.milliseconds * 100.0;
    }

    if (baseline.sheets > 0) {
        delta.sheetReductionPercent =
            (static_cast<double>(baseline.sheets) -
             static_cast<double>(optimized.sheets)) /
            static_cast<double>(baseline.sheets) * 100.0;
    }

    delta.utilizationImprovementPercentagePoints =
        delta.utilizationPercentagePoints;

    return delta;
}

} // namespace

BenchmarkResult benchmarkNest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& optimizedOptions
) {
    Options baseline = optimizedOptions;

    // Baseline is intentionally a true greedy reference:
    // one deterministic nesting pass with post-optimization and recovery
    // disabled. Physical constraints remain identical to the optimized case.
    baseline.iterations = 1;
    baseline.enableOptimizer = false;
    baseline.enableProductionValidation = false;
    baseline.enableAutoRepair = false;
    baseline.enableAdaptiveDestroyRepair = false;
    baseline.autoRepairAttempts = 0;
    baseline.autoRepairTimeBudgetMs = 0;
    baseline.adaptiveRepairAttempts = 0;
    baseline.adaptiveRepairMaxNeighbors = 0;
    baseline.adaptiveRepairRounds = 0;
    baseline.enableSmallPartOptimization = false;
    baseline.smallPartRefillPasses = 1;
    baseline.residualRetryPasses = 1;

    BenchmarkResult result;
    result.baseline = runCase(
        "Базовый поиск (1 итерация)",
        instances,
        sheet,
        baseline
    );
    result.optimized = runCase(
        "Оптимизированный поиск",
        instances,
        sheet,
        optimizedOptions
    );
    result.delta = makeDelta(
        result.baseline,
        result.optimized
    );

    return result;
}

} // namespace sheetnest
