#pragma once

#include "cutting.hpp"
#include "dxf.hpp"
#include "dxf_export.hpp"
#include "nesting.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sheetnest {

enum class PipelineStatus {
    Pending,
    Ok,
    Fail,
    Skipped
};

struct PipelineStageResult {
    std::string stage;
    PipelineStatus status{PipelineStatus::Pending};
    std::string reason;
    std::size_t metricA{};
    std::size_t metricB{};
    double metricValue{};
};

struct ProductionPipelineReport {
    std::vector<PipelineStageResult> stages;
    std::string failedStage;
    std::string failureReason;
    std::size_t expectedInstances{};
    std::size_t placedInstances{};
    std::size_t unplacedInstances{};
    std::size_t sheets{};
    std::size_t camOperations{};
    std::size_t exportedDxfBytes{};
    bool roundTripValid{};
    bool complete{};
};

ProductionPipelineReport validateProductionPipeline(
    const std::string& sourceDxf,
    const Sheet& sheet,
    const Options& options,
    const CuttingParameters& technology,
    const DxfExportOptions& exportOptions = {}
);

const char* toString(PipelineStatus status);

} // namespace sheetnest
