#include "sheetnest/diagnostics.hpp"

#include <unordered_set>
#include <string>

namespace sheetnest {

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

} // namespace sheetnest
