#include "sheetnest/diagnostics.hpp"

#include <unordered_set>
#include <string>
#include <unordered_map>

namespace sheetnest {
namespace {

const char* failureReasonCode(NestingFailureReason reason) {
    switch (reason) {
    case NestingFailureReason::None: return "None";
    case NestingFailureReason::NoFeasiblePosition: return "NoFeasiblePosition";
    case NestingFailureReason::Timeout: return "Timeout";
    case NestingFailureReason::InvalidGeometry: return "InvalidGeometry";
    case NestingFailureReason::RepairExhausted: return "RepairExhausted";
    case NestingFailureReason::Cancelled: return "Cancelled";
    }
    return "None";
}

} // namespace

std::vector<InstanceDiagnostic> diagnoseNest(
    const std::vector<Instance>& instances,
    const Result& result
) {
    std::unordered_set<std::string> placed;
    for (const auto& sheet : result.sheets) {
        for (const auto& placement : sheet) {
            placed.insert(placement.id);
        }
    }

    std::unordered_set<std::string> unplaced(
        result.unplaced.begin(),
        result.unplaced.end()
    );

    std::vector<InstanceDiagnostic> diagnostics;
    diagnostics.reserve(instances.size());

    for (const auto& instance : instances) {
        InstanceDiagnostic diagnostic;
        diagnostic.instanceId = instance.id;
        diagnostic.unitId = instance.unitId.empty()
            ? instance.id + ":unit-1"
            : instance.unitId;
        diagnostic.sourceId = instance.part.sourceId;
        diagnostic.layer = instance.part.layer;

        if (placed.find(instance.id) != placed.end()) {
            diagnostic.status = InstanceDiagnosticStatus::Placed;
            diagnostic.stage = "nesting/placed";
            diagnostic.message =
                "Деталь размещена в итоговой раскладке.";
        } else if (unplaced.find(instance.id) != unplaced.end()) {
            diagnostic.status = InstanceDiagnosticStatus::Unplaced;
            diagnostic.stage = "nesting/no-valid-candidate";
            diagnostic.message =
                "Не найдено допустимое размещение после проверки "
                "поворотов, NFP, границ листа и зазора.";
        } else {
            diagnostic.status = InstanceDiagnosticStatus::Unknown;
            diagnostic.stage = "diagnostics/unknown";
            diagnostic.message =
                "Экземпляр отсутствует и в размещённых, и в списке unplaced.";
        }

        diagnostics.push_back(std::move(diagnostic));
    }

    return diagnostics;
}

void enrichDiagnostics(
    std::vector<InstanceDiagnostic>& diagnostics,
    const CuttingPath& route,
    const ProductionValidationReport& validation,
    const Result* nestingResult
) {
    std::unordered_map<std::string, std::size_t> operationCounts;
    std::unordered_map<std::string, double> operationSeconds;
    std::unordered_map<std::string, std::size_t> sheets;
    for (const auto& operation : route.operations) {
        ++operationCounts[operation.instanceId];
        operationSeconds[operation.instanceId] += operation.totalSeconds;
        sheets[operation.instanceId] = operation.sheetIndex;
    }

    for (auto& diagnostic : diagnostics) {
        const auto opCount = operationCounts.find(diagnostic.instanceId);
        if (opCount != operationCounts.end()) {
            diagnostic.cuttingOperationCount = opCount->second;
            diagnostic.cuttingSeconds = operationSeconds[diagnostic.instanceId];
            diagnostic.sheetIndex = sheets[diagnostic.instanceId];
        }

        diagnostic.repairAttempts = validation.repairAttempts;
        diagnostic.adaptiveRepairRounds = validation.adaptiveRepairRounds;

        diagnostic.candidateChecks = 0;
        diagnostic.nfpChecks = 0;
        diagnostic.nfpTimeouts = 0;
        diagnostic.nfpFallbacks = 0;
        diagnostic.nestingElapsedMs = 0;
        diagnostic.repairRounds = 0;
        diagnostic.repairConflictRounds = 0;
        diagnostic.repairExtracted = false;
        diagnostic.repairMoved = false;
        diagnostic.failureReason = "None";

        if (nestingResult) {
            for (const auto& telemetry : nestingResult->instanceTelemetry) {
                if (telemetry.instanceId != diagnostic.instanceId) continue;
                diagnostic.candidateChecks = telemetry.candidateChecks;
                diagnostic.nfpChecks = telemetry.nfpChecks;
                diagnostic.nfpTimeouts = telemetry.nfpTimeouts;
                diagnostic.nfpFallbacks = telemetry.nfpFallbacks;
                diagnostic.nestingElapsedMs = telemetry.elapsedMs;
                diagnostic.repairRounds = telemetry.repairRounds;
                diagnostic.repairConflictRounds = telemetry.repairConflictRounds;
                diagnostic.repairExtracted = telemetry.repairExtracted;
                diagnostic.repairMoved = telemetry.repairMoved;
                diagnostic.failureReason = failureReasonCode(telemetry.reason);
                break;
            }
        }

        if (diagnostic.status == InstanceDiagnosticStatus::Placed) {
            diagnostic.finalStatus = "Placed";
        } else if (diagnostic.status == InstanceDiagnosticStatus::Unplaced) {
            diagnostic.finalStatus = validation.valid
                ? "Unplaced after valid nesting"
                : "Unplaced / validation not valid";
        } else {
            diagnostic.finalStatus = "Unknown";
        }
    }
}

} // namespace sheetnest
