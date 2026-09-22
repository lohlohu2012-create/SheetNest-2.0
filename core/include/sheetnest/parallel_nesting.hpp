#pragma once

#include "nesting.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace sheetnest {

enum class NestingProgressPhase {
    Starting,
    WorkerStarted,
    IterationFinished,
    CandidatesCollected,
    GlobalOptimization,
    Completed,
    Cancelled,
    TimedOut
};

struct NestingProgress {
    NestingProgressPhase phase{NestingProgressPhase::Starting};
    std::size_t workerIndex{};
    std::size_t workerCount{};
    std::size_t completedIterations{};
    std::size_t totalIterations{};
    std::size_t placed{};
    std::size_t skipped{};
    std::size_t sheets{};
    double utilization{};
    std::uint64_t elapsedMs{};
    std::uint64_t remainingMs{};
    std::string message;
};

class NestingCandidateCollector {
public:
    explicit NestingCandidateCollector(
        std::size_t capacity = 8
    );

    void add(Result candidate);
    std::vector<Result> snapshot() const;
    std::size_t size() const;

private:
    std::size_t capacity_{8};
    mutable std::mutex mutex_;
    std::vector<Result> candidates_;
};

struct ParallelNestingOptions {
    std::size_t workers{};
    std::size_t iterations{24};
    std::size_t candidateCapacity{8};
    std::uint64_t timeBudgetMs{120000};
    std::uint32_t seed{0x534E4553u};
    std::function<void(const NestingProgress&)> onProgress;
};

class ParallelNestingController {
public:
    ParallelNestingController();
    ~ParallelNestingController();

    ParallelNestingController(
        const ParallelNestingController&
    ) = delete;

    ParallelNestingController& operator=(
        const ParallelNestingController&
    ) = delete;

    void requestCancel();
    bool cancelRequested() const;

    Result run(
        const std::vector<Instance>& instances,
        const Sheet& sheet,
        const Options& nestingOptions,
        const ParallelNestingOptions& options
    );

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sheetnest
