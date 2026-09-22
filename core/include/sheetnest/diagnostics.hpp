#pragma once

#include "nesting.hpp"
#include "cutting_path.hpp"
#include "production_validation.hpp"

#include <vector>
#include <string>

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
    std::size_t nfpTimeouts{};
    std::size_t nfpFallbacks{};
    std::string finalStatus;
};

std::vector<InstanceDiagnostic> diagnoseNest(
    const std::vector<Instance>& instances,
    const Result& result
);

void enrichDiagnostics(
    std::vector<InstanceDiagnostic>& diagnostics,
    const CuttingPath& route,
    const ProductionValidationReport& validation
);

} // namespace sheetnest
