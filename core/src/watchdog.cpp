#include "sheetnest/watchdog.hpp"

#include <chrono>
#include <algorithm>

namespace sheetnest {
namespace {

std::int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

} // namespace

Watchdog::Watchdog(double timeoutMs, double stallMs)
    : timeoutMs_(std::max(0.0, timeoutMs)),
      stallMs_(std::max(0.0, stallMs)),
      startedNs_(nowNs()),
      lastHeartbeatNs_(startedNs_) {}

void Watchdog::stage(
    WatchdogStage stage,
    const std::string& message
) {
    std::lock_guard<std::mutex> lock(mutex_);\n    stage_ = stage;
    message_ = message;
    heartbeat_();
    lastHeartbeatNs_ = nowNs();
}

void Watchdog::heartbeat(
    std::size_t completedItems,
    std::size_t totalItems,
    const std::string& message
) {
    std::lock_guard<std::mutex> lock(mutex_);\n    completedItems_.store(completedItems);
    totalItems_.store(totalItems);
    if (!message.empty()) message_ = message;
    heartbeat_++;
    lastHeartbeatNs_ = nowNs();
}

WatchdogSnapshot Watchdog::poll() {
    const auto now = nowNs();
    const double elapsed =
        static_cast<double>(now - startedNs_) / 1000000.0;
    const double sinceHeartbeat =
        static_cast<double>(now - lastHeartbeatNs_) / 1000000.0;

    if (stopRequested_.load()) {
        state_ = WatchdogState::Stopped;
    } else if (timeoutMs_ > 0.0 && elapsed >= timeoutMs_) {
        state_ = WatchdogState::Timeout;
        stopRequested_.store(true);
    } else if (stallMs_ > 0.0 && sinceHeartbeat >= stallMs_) {
        state_ = WatchdogState::Stalled;
    } else if (state_ != WatchdogState::Completed) {
        state_ = WatchdogState::Running;
    }

    WatchdogSnapshot result;
    result.stage = stage_;
    result.state = state_;
    result.completedItems = completedItems_.load();
    result.totalItems = totalItems_.load();
    result.heartbeat = heartbeat_.load();
    result.elapsedMs = elapsed;
    result.remainingMs =
        timeoutMs_ > 0.0
            ? std::max(0.0, timeoutMs_ - elapsed)
            : 0.0;
    result.message = message_;
    return result;
}

void Watchdog::requestStop() {
    stopRequested_.store(true);
    state_ = WatchdogState::Stopped;
}

bool Watchdog::stopRequested() const {
    return stopRequested_.load();
}

WatchdogSnapshot Watchdog::snapshot() const {
    return const_cast<Watchdog*>(this)->poll();
}

const char* toString(WatchdogStage stage) {
    switch (stage) {
    case WatchdogStage::Starting: return "Starting";
    case WatchdogStage::Worker: return "Worker";
    case WatchdogStage::NfpSearch: return "NFP Search";
    case WatchdogStage::CandidateCollector: return "Candidate Collector";
    case WatchdogStage::GlobalOptimization: return "Global Optimization";
    case WatchdogStage::ProductionValidator: return "Production Validator";
    case WatchdogStage::AdaptiveRepair: return "Adaptive Repair";
    case WatchdogStage::Finalizing: return "Finalizing";
    case WatchdogStage::Completed: return "Completed";
    }
    return "Unknown";
}

const char* toString(WatchdogState state) {
    switch (state) {
    case WatchdogState::Running: return "RUNNING";
    case WatchdogState::Stalled: return "STALLED";
    case WatchdogState::Timeout: return "TIMEOUT";
    case WatchdogState::Completed: return "COMPLETED";
    case WatchdogState::Stopped: return "STOPPED";
    }
    return "UNKNOWN";
}

} // namespace sheetnest
