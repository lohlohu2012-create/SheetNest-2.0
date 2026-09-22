#pragma once
#include "geometry.hpp"

#include <cstddef>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
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

struct NestingStats {
    std::size_t candidateChecks{};
    std::size_t collisionChecks{};
    std::size_t nfpChecks{};
    std::size_t refillMoves{};
    std::size_t exchangeAttempts{};
    std::size_t sheetsEliminated{};
    std::size_t optimizerPasses{};
};

struct Result {
    std::vector<std::vector<Placement>> sheets;
    std::vector<std::string> unplaced;
    double utilization{};
    NestingStats stats{};
    bool productionValidated{};
    bool productionValid{};
    std::size_t productionIssueCount{};
};

struct NestingRunControl {
    std::atomic<bool> cancelRequested{false};
    mutable std::atomic<bool> timeoutObserved{false};
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max();

    bool shouldStop() const {
        if (cancelRequested.load(std::memory_order_relaxed)) {
            return true;
        }

        if (std::chrono::steady_clock::now() >= deadline) {
            timeoutObserved.store(
                true,
                std::memory_order_relaxed
            );
            return true;
        }

        return false;
    }
};

struct Options {
    std::vector<int> rotations{0, 90, 180, 270};
    std::size_t iterations{24};
    double gapMm{2.0};
    std::uint32_t seed{0x534E4553u};
    bool enableOptimizer{true};
    bool enableProductionValidation{true};
    bool enableAutoRepair{true};
    std::size_t autoRepairAttempts{8};
    std::uint64_t autoRepairTimeBudgetMs{15000};
    bool enableAdaptiveDestroyRepair{true};
    std::size_t adaptiveRepairAttempts{4};
    std::size_t adaptiveRepairMaxNeighbors{16};
    std::size_t adaptiveRepairRounds{2};
    std::shared_ptr<NestingRunControl> control{};
};

Result nest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options
);

// Runs the global post-processing stage on an already-built candidate.
// The stage is transactional: invalid repacks/exchanges are rolled back.
bool optimizeNestingResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    Result& result,
    std::vector<std::string>* extractedIdsOut = nullptr
);

// Locally removes only a conflict-driven group of placements, repacks that
// group into the affected sheet regions while keeping unrelated placements
// fixed, and leaves the caller to run the final Production Validator.
bool adaptiveDestroyAndRepairResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    const std::vector<std::string>& seedIds,
    Result& result
);

} // namespace sheetnest
