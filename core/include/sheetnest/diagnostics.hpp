#pragma once

#include "nesting.hpp"

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
};

std::vector<InstanceDiagnostic> diagnoseNest(
    const std::vector<Instance>& instances,
    const Result& result
);

} // namespace sheetnest
