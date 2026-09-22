#include "sheetnest/parallel_nesting.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

namespace sheetnest {
namespace {

using Clock = std::chrono::steady_clock;

struct ControlState {
    std::atomic<bool> cancelRequested{false};
    std::atomic<bool> timeoutObserved{false};
    Clock::time_point deadline{};
};

bool betterResult(const Result& candidate, const Result& best) {
    if (candidate.unplaced.size() != best.unplaced.size()) {
        return candidate.unplaced.size() < best.unplaced.size();
    }
    if (candidate.sheets.size() != best.sheets.size()) {
        return candidate.sheets.size() < best.sheets.size();
    }
    return candidate.utilization > best.utilization + 1e-7;
}

const char* phaseMessage(NestingProgressPhase phase) {
    switch (phase) {
    case NestingProgressPhase::Starting: return "Запуск";
    case NestingProgressPhase::WorkerStarted: return "Worker запущен";
    case NestingProgressPhase::IterationFinished: return "Итерация завершена";
    case NestingProgressPhase::Completed: return "Расчёт завершён";
    case NestingProgressPhase::Cancelled: return "Расчёт отменён";
    case NestingProgressPhase::TimedOut: return "Достигнут лимит времени";
    }
    return "Расчёт";
}

} // namespace

struct ParallelNestingController::Impl {
    std::shared_ptr<ControlState> control =
        std::make_shared<ControlState>();

    void reset(std::uint64_t timeBudgetMs) {
        control = std::make_shared<ControlState>();
        control->deadline =
            Clock::now() +
            std::chrono::milliseconds(timeBudgetMs);
    }
};

ParallelNestingController::ParallelNestingController()
    : impl_(std::make_unique<Impl>()) {}

ParallelNestingController::~ParallelNestingController() = default;

void ParallelNestingController::requestCancel() {
    if (impl_ && impl_->control) {
        impl_->control->cancelRequested.store(
            true,
            std::memory_order_relaxed
        );
    }
}

bool ParallelNestingController::cancelRequested() const {
    return impl_ &&
           impl_->control &&
           impl_->control->cancelRequested.load(
               std::memory_order_relaxed
           );
}

Result ParallelNestingController::run(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& nestingOptions,
    const ParallelNestingOptions& options
) {
    const std::size_t totalIterations =
        std::max<std::size_t>(1, options.iterations);

    const unsigned int hardware =
        std::max(1u, std::thread::hardware_concurrency());

    const std::size_t workerCount =
        std::max<std::size_t>(
            1,
            std::min(
                totalIterations,
                options.workers == 0
                    ? static_cast<std::size_t>(hardware)
                    : options.workers
            )
        );

    impl_->reset(options.timeBudgetMs);
    const auto control = impl_->control;
    const auto started = Clock::now();

    Result best;
    best.utilization = -1.0;
    best.unplaced.reserve(instances.size());
    for (const auto& instance : instances) {
        best.unplaced.push_back(instance.id);
    }

    std::mutex resultMutex;
    std::mutex progressMutex;
    std::atomic<std::size_t> nextIteration{0};
    std::atomic<std::size_t> completedIterations{0};

    auto publish = [&](NestingProgress event) {
        if (!options.onProgress) return;

        const auto now = Clock::now();
        event.elapsedMs =
            static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - started
                ).count()
            );

        if (control->deadline > now) {
            event.remainingMs =
                static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        control->deadline - now
                    ).count()
                );
        } else {
            event.remainingMs = 0;
        }

        if (event.message.empty()) {
            event.message = phaseMessage(event.phase);
        }

        std::lock_guard<std::mutex> lock(progressMutex);
        options.onProgress(event);
    };

    publish({
        NestingProgressPhase::Starting,
        0,
        workerCount,
        0,
        totalIterations,
        0,
        instances.size(),
        0,
        0.0,
        0,
        options.timeBudgetMs,
        "Запуск calculation controller"
    });

    auto worker = [&](std::size_t workerIndex) {
        while (true) {
            if (control->cancelRequested.load(
                    std::memory_order_relaxed)) {
                break;
            }

            if (Clock::now() >= control->deadline) {
                control->timeoutObserved.store(
                    true,
                    std::memory_order_relaxed
                );
                break;
            }

            const std::size_t iteration =
                nextIteration.fetch_add(
                    1,
                    std::memory_order_relaxed
                );

            if (iteration >= totalIterations) break;

            publish({
                NestingProgressPhase::WorkerStarted,
                workerIndex,
                workerCount,
                completedIterations.load(
                    std::memory_order_relaxed
                ),
                totalIterations,
                0,
                instances.size(),
                0,
                0.0,
                0,
                0,
                "Worker " +
                    std::to_string(workerIndex + 1) +
                    " → итерация " +
                    std::to_string(iteration + 1)
            });

            Options iterationOptions = nestingOptions;
            iterationOptions.iterations = 1;
            iterationOptions.seed =
                options.seed +
                static_cast<std::uint32_t>(
                    iteration * 0x9E3779B9u
                ) +
                static_cast<std::uint32_t>(workerIndex);
            iterationOptions.control = control;

            const Result candidate =
                nest(
                    instances,
                    sheet,
                    iterationOptions
                );

            {
                std::lock_guard<std::mutex> lock(resultMutex);
                if (best.utilization < 0.0 ||
                    betterResult(candidate, best)) {
                    best = candidate;
                }
            }

            const auto completed =
                completedIterations.fetch_add(
                    1,
                    std::memory_order_relaxed
                ) + 1;

            std::size_t bestSheets{};
            std::size_t bestSkipped{};
            double bestUtilization{};

            {
                std::lock_guard<std::mutex> lock(resultMutex);
                bestSheets = best.sheets.size();
                bestSkipped = best.unplaced.size();
                bestUtilization = best.utilization;
            }

            publish({
                NestingProgressPhase::IterationFinished,
                workerIndex,
                workerCount,
                completed,
                totalIterations,
                instances.size() - bestSkipped,
                bestSkipped,
                bestSheets,
                bestUtilization,
                0,
                0,
                "Итерация " +
                    std::to_string(iteration + 1) +
                    "/" +
                    std::to_string(totalIterations) +
                    "; лучший результат: " +
                    std::to_string(bestSheets) +
                    " листов"
            });
        }
    };

    std::vector<std::thread> workers;
    workers.reserve(workerCount);

    for (std::size_t i = 0; i < workerCount; ++i) {
        workers.emplace_back(worker, i);
    }

    for (auto& thread : workers) {
        if (thread.joinable()) thread.join();
    }

    if (best.utilization < 0.0) {
        best.utilization = 0.0;
    }

    if (control->timeoutObserved.load(
            std::memory_order_relaxed
        ) && !control->cancelRequested.load(
            std::memory_order_relaxed
        )) {
        publish({
            NestingProgressPhase::TimedOut,
            0,
            workerCount,
            completedIterations.load(
                std::memory_order_relaxed
            ),
            totalIterations,
            instances.size() - best.unplaced.size(),
            best.unplaced.size(),
            best.sheets.size(),
            best.utilization,
            0,
            0,
            "Лимит времени достигнут; возвращён лучший найденный результат"
        });
    } else if (control->cancelRequested.load(
                   std::memory_order_relaxed
               )) {
        publish({
            NestingProgressPhase::Cancelled,
            0,
            workerCount,
            completedIterations.load(
                std::memory_order_relaxed
            ),
            totalIterations,
            instances.size() - best.unplaced.size(),
            best.unplaced.size(),
            best.sheets.size(),
            best.utilization,
            0,
            0,
            "Расчёт остановлен пользователем"
        });
    } else {
        publish({
            NestingProgressPhase::Completed,
            0,
            workerCount,
            completedIterations.load(
                std::memory_order_relaxed
            ),
            totalIterations,
            instances.size() - best.unplaced.size(),
            best.unplaced.size(),
            best.sheets.size(),
            best.utilization,
            0,
            0,
            "Все итерации завершены"
        });
    }

    return best;
}

} // namespace sheetnest
