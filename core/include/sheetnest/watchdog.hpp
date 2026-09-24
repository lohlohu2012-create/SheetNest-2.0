#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>\n#include <mutex>

namespace sheetnest {

enum class WatchdogStage {
    Starting,
    Worker,
    NfpSearch,
    CandidateCollector,
    GlobalOptimization,
    ProductionValidator,
    AdaptiveRepair,
    Finalizing,
    Completed
};

enum class WatchdogState {
    Running,
    Stalled,
    Timeout,
    Completed,
    Stopped
};

struct WatchdogSnapshot {
    WatchdogStage stage{WatchdogStage::Starting};
    WatchdogState state{WatchdogState::Running};
    std::size_t completedItems{};
    std::size_t totalItems{};
    std::uint64_t heartbeat{};
    double elapsedMs{};
    double remainingMs{};
    std::string message;
};

class Watchdog {
public:
    explicit Watchdog(
        double timeoutMs = 120000.0,
        double stallMs = 5000.0
    );

    void stage(WatchdogStage stage, const std::string& message = {});
    void heartbeat(
        std::size_t completedItems = 0,
        std::size_t totalItems = 0,
        const std::string& message = {}
    );
    WatchdogSnapshot poll();
    void requestStop();
    bool stopRequested() const;
    WatchdogSnapshot snapshot() const;

private:
    mutable std::atomic<std::uint64_t> heartbeat_{0};
    std::atomic<bool> stopRequested_{false};
    std::atomic<std::size_t> completedItems_{0};
    std::atomic<std::size_t> totalItems_{0};
    WatchdogStage stage_{WatchdogStage::Starting};
    WatchdogState state_{WatchdogState::Running};
    double timeoutMs_{};
    double stallMs_{};
    std::uint64_t lastHeartbeat_{};
    std::string message_;
    std::int64_t startedNs_{};
    std::int64_t lastHeartbeatNs_{};
};

const char* toString(WatchdogStage stage);
const char* toString(WatchdogState state);

} // namespace sheetnest
