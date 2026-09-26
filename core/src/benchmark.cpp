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

    benchmark.nfpCacheHitRate =
        (benchmark.nfpCacheHits + benchmark.nfpCacheMisses) > 0
            ? static_cast<double>(benchmark.nfpCacheHits) /
              static_cast<double>(benchmark.nfpCacheHits + benchmark.nfpCacheMisses)
            : 0.0;
    benchmark.nfpAttemptsPerPlaced =
        benchmark.placed > 0
            ? static_cast<double>(benchmark.nfpAttempts) /
              static_cast<double>(benchmark.placed)
            : 0.0;
    benchmark.millisecondsPerPlaced =
        benchmark.placed > 0
            ? benchmark.milliseconds / static_cast<double>(benchmark.placed)
            : benchmark.milliseconds;

    benchmark.initiallyPlaced = result.stats.recovery.initiallyPlaced;
    benchmark.recoveredBySmallPart = result.stats.recovery.recoveredBySmallPart;
    benchmark.recoveredByResidualRetry = result.stats.recovery.recoveredByResidualRetry;
    benchmark.recoveredOnNewSheet = result.stats.recovery.recoveredOnNewSheet;
    benchmark.recoveredByOptimizer = result.stats.recovery.recoveredByOptimizer;
    benchmark.recoveredByAdaptiveRepair = result.stats.recovery.recoveredByAdaptiveRepair;
    benchmark.finallyUnplaced = result.stats.recovery.finallyUnplaced;

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
    delta.nfpCacheHitRatePercentagePoints =
        (optimized.nfpCacheHitRate - baseline.nfpCacheHitRate) * 100.0;
    delta.nfpAttemptsPerPlacedDelta =
        optimized.nfpAttemptsPerPlaced - baseline.nfpAttemptsPerPlaced;
    delta.millisecondsPerPlacedDelta =
        optimized.millisecondsPerPlaced - baseline.millisecondsPerPlaced;
    delta.optimizedPlacementSafetyPassed =
        optimized.placed >= baseline.placed;

    return delta;
}

} // namespace

const char* benchmarkBottleneckName(BenchmarkBottleneck bottleneck) {
    switch (bottleneck) {
        case BenchmarkBottleneck::None: return "None";
        case BenchmarkBottleneck::PlacementSafety: return "PlacementSafety";
        case BenchmarkBottleneck::InvalidGeometry: return "InvalidGeometry";
        case BenchmarkBottleneck::NfpTimeout: return "NfpTimeout";
        case BenchmarkBottleneck::NoFeasiblePosition: return "NoFeasiblePosition";
        case BenchmarkBottleneck::CandidateSearch: return "CandidateSearch";
        case BenchmarkBottleneck::SheetCapacity: return "SheetCapacity";
        case BenchmarkBottleneck::RecoveryPressure: return "RecoveryPressure";
    }
    return "None";
}

BenchmarkAnalysis analyzeBenchmark(const BenchmarkResult& result) {
    BenchmarkAnalysis analysis;
    const auto& b = result.baseline;
    const auto& o = result.optimized;
    const auto& d = result.delta;

    analysis.placementSafetyPassed = d.optimizedPlacementSafetyPassed;
    analysis.lossFree = o.finallyUnplaced == 0 && o.skipped == 0;
    analysis.timeImproved = o.milliseconds < b.milliseconds;
    analysis.sheetsReduced = o.sheets < b.sheets;
    analysis.utilizationImproved =
        o.utilization > b.utilization + 1e-12;
    analysis.nfpLoadReduced =
        o.nfpAttemptsPerPlaced + 1e-12 < b.nfpAttemptsPerPlaced;

    // Safety and geometry diagnostics have priority over performance
    // classifications: a faster run that loses instances is not a
    // successful production benchmark.
    if (!analysis.placementSafetyPassed) {
        analysis.bottleneck = BenchmarkBottleneck::PlacementSafety;
        analysis.summary = "Оптимизация снизила число размещённых деталей.";
    } else if (o.telemetryInvalidGeometry > b.telemetryInvalidGeometry) {
        analysis.bottleneck = BenchmarkBottleneck::InvalidGeometry;
        analysis.summary = "Рост отказов связан с некорректной геометрией.";
    } else {
        const double timeoutBase =
            b.nfpAttempts > 0
                ? static_cast<double>(b.nfpTimeouts) /
                  static_cast<double>(b.nfpAttempts)
                : 0.0;
        const double timeoutOptimized =
            o.nfpAttempts > 0
                ? static_cast<double>(o.nfpTimeouts) /
                  static_cast<double>(o.nfpAttempts)
                : 0.0;

        if (o.nfpTimeouts > b.nfpTimeouts &&
            timeoutOptimized > timeoutBase + 0.01) {
            analysis.bottleneck = BenchmarkBottleneck::NfpTimeout;
            analysis.summary = "Основное узкое место: тайм-ауты NFP.";
        } else if (
            o.nfpAttemptsPerPlaced >
                std::max(10.0, b.nfpAttemptsPerPlaced * 1.25) ||
            o.candidateChecks >
                std::max<std::size_t>(
                    1000,
                    b.candidateChecks +
                        b.candidateChecks / 4
                )
        ) {
            analysis.bottleneck = BenchmarkBottleneck::CandidateSearch;
            analysis.summary = "Основное узкое место: избыточный поиск кандидатов.";
        } else if (
            o.telemetryNoFeasiblePosition > b.telemetryNoFeasiblePosition
        ) {
            analysis.bottleneck = BenchmarkBottleneck::NoFeasiblePosition;
            analysis.summary = "Основное узкое место: отсутствие допустимой позиции.";
        } else if (
            o.sheets > b.sheets &&
            o.utilization + 0.01 < b.utilization
        ) {
            analysis.bottleneck = BenchmarkBottleneck::SheetCapacity;
            analysis.summary = "Основное узкое место: ёмкость листа и остаточное пространство.";
        } else if (o.finallyUnplaced > 0) {
            analysis.bottleneck = BenchmarkBottleneck::RecoveryPressure;
            analysis.summary = "Остались неразмещённые детали после recovery.";
        } else {
            analysis.bottleneck = BenchmarkBottleneck::None;
            analysis.summary = "Явного узкого места по текущей телеметрии не выявлено.";
        }
    }

    analysis.bottleneckName = benchmarkBottleneckName(analysis.bottleneck);
    return analysis;
}

std::vector<BenchmarkMatrixEntry> benchmarkMatrix(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& optimizedOptions
) {
    std::vector<BenchmarkMatrixEntry> matrix;

    auto addCase = [&](const std::string& mode, Options options) {
        BenchmarkMatrixEntry entry;
        entry.mode = mode;
        entry.result = runCase(mode, instances, sheet, options);
        entry.placementSafetyPassed =
            entry.result.placed >= instances.size();
        matrix.push_back(std::move(entry));
    };

    Options greedy = optimizedOptions;
    greedy.iterations = 1;
    greedy.enableOptimizer = false;
    greedy.enableProductionValidation = false;
    greedy.enableAutoRepair = false;
    greedy.enableAdaptiveDestroyRepair = false;
    greedy.enableSmallPartOptimization = false;
    greedy.autoRepairAttempts = 0;
    greedy.adaptiveRepairAttempts = 0;
    greedy.adaptiveRepairMaxNeighbors = 0;
    greedy.adaptiveRepairRounds = 0;
    addCase("Greedy", greedy);

    Options search = optimizedOptions;
    search.enableOptimizer = false;
    search.enableProductionValidation = false;
    search.enableAutoRepair = false;
    search.enableAdaptiveDestroyRepair = false;
    search.autoRepairAttempts = 0;
    search.adaptiveRepairAttempts = 0;
    search.adaptiveRepairMaxNeighbors = 0;
    search.adaptiveRepairRounds = 0;
    addCase("Multi-iteration", search);

    Options refill = search;
    refill.enableSmallPartOptimization = true;
    addCase("Small-part refill", refill);

    Options recovery = refill;
    recovery.enableAutoRepair = true;
    recovery.autoRepairAttempts = std::max<std::size_t>(
        1, optimizedOptions.autoRepairAttempts
    );
    addCase("Recovery", recovery);

    Options full = optimizedOptions;
    addCase("Full optimized", full);

    // Analyze every mode against the same greedy reference.
    const BenchmarkCase& reference = matrix.front().result;
    for (auto& entry : matrix) {
        BenchmarkResult pair;
        pair.baseline = reference;
        pair.optimized = entry.result;
        pair.delta = makeDelta(reference, entry.result);
        pair.analysis = analyzeBenchmark(pair);
        entry.analysis = pair.analysis;
        entry.placementSafetyPassed =
            pair.delta.optimizedPlacementSafetyPassed;
    }

    return matrix;
}

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
    result.analysis = analyzeBenchmark(result);
    result.matrix = benchmarkMatrix(instances, sheet, optimizedOptions);

    return result;
}

} // namespace sheetnest
