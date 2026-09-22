#include "sheetnest/benchmark.hpp"

#include <chrono>

namespace sheetnest {
namespace {

BenchmarkCase runCase(
    const std::string& name,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options
) {
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
    benchmark.placed = instances.size() - benchmark.skipped;
    benchmark.utilization = result.utilization;
    benchmark.candidateChecks = result.stats.candidateChecks;
    benchmark.collisionChecks = result.stats.collisionChecks;
    benchmark.nfpChecks = result.stats.nfpChecks;
    benchmark.refillMoves = result.stats.refillMoves;
    benchmark.exchangeAttempts = result.stats.exchangeAttempts;
    benchmark.sheetsEliminated = result.stats.sheetsEliminated;
    benchmark.optimizerPasses = result.stats.optimizerPasses;
    return benchmark;
}

} // namespace

BenchmarkResult benchmarkNest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& optimizedOptions
) {
    Options baseline = optimizedOptions;

    // Baseline is intentionally a true greedy reference:
    // one deterministic nesting pass with post-optimization and automatic
    // repair disabled. Geometry, sheet, gap and permitted rotations remain
    // identical so the delta measures search/optimization work rather than
    // different manufacturing constraints.
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
    return result;
}

} // namespace sheetnest
