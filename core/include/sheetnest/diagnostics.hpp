#pragma once

#include "nesting.hpp"
#include "cutting_path.hpp"
#include "production_validation.hpp"

#include <vector>
#include <string>
#include <cstdint>

namespace sheetnest {

enum class InstanceDiagnosticStatus {
    Placed,
    Unplaced,
    Unknown
};

struct InstanceDiagnostic {
    std::string instanceId;
    std::string unitId;
    std::string sourceId;
    std::string layer;
    InstanceDiagnosticStatus status{InstanceDiagnosticStatus::Unknown};
    std::string stage;
    std::string message;
    std::size_t sheetIndex{static_cast<std::size_t>(-1)};
    std::size_t cuttingOperationCount{};
    double cuttingSeconds{};
    std::size_t repairAttempts{};
    std::size_t adaptiveRepairRounds{};
    std::size_t candidateChecks{};
    std::size_t nfpChecks{};
    std::size_t nfpTimeouts{};
    std::size_t nfpFallbacks{};
    std::size_t nfpTimeoutFallbacks{};
    std::size_t nfpCacheHits{};
    std::size_t nfpCacheMisses{};
    std::size_t boundsRejections{};
    std::size_t collisionRejections{};
    std::size_t feasibleCandidates{};
    std::uint64_t nestingElapsedMs{};
    std::size_t repairRounds{};
    std::size_t repairConflictRounds{};
    bool repairExtracted{};
    bool repairMoved{};
    std::string failureReason;
    std::string finalStatus;
};

std::vector<InstanceDiagnostic> diagnoseNest(
    const std::vector<Instance>& instances,
    const Result& result
);

void enrichDiagnostics(
    std::vector<InstanceDiagnostic>& diagnostics,
    const CuttingPath& route,
    const ProductionValidationReport& validation,
    const Result* nestingResult = nullptr
);

} // namespace sheetnest
