#pragma once

#include "geometry.hpp"
#include "nesting.hpp"
#include "cutting_path.hpp"
#include "cam_export.hpp"
#include "dxf_model.hpp"
#include "dxf_export.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sheetnest {

enum class ProductionPipelineStage {
    DxfParse,
    DxfPreflight,
    Nesting,
    AdaptiveRepair,
    Coverage,
    CamRoute,
    CamValidation,
    DxfExport,
    DxfRoundTrip,
    Complete,
    Failed
};

const char* productionPipelineStageName(ProductionPipelineStage stage);
struct ProductionPipelineReport {
    bool valid{false};
    ProductionPipelineStage failedStage{ProductionPipelineStage::Failed};
    std::string failureReason;
    std::vector<ProductionPipelineStage> completedStages;
    DxfPreflightReport dxfPreflight;
    ProductionValidationReport nestingValidation;
    CamValidationReport camValidation;
    std::size_t camOperationCount{};
    std::size_t exportedBytes{};
    bool roundTripValid{false};
    DxfPreflightReport roundTripPreflight;
    std::string exportedDxf;
};

ProductionPipelineReport validateProductionPipeline(
    const DxfDocument& document,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    const Result& result,
    const CuttingParameters& cuttingParameters,
    const PathOptions& pathOptions = {},
    const CamExportOptions& camOptions = {},
    const DxfExportOptions& dxfOptions = {}
);


enum class ProductionValidationIssueType {
    Collision,
    Gap,
    Margin,
    DuplicateId,
    MissingId,
    UnknownId
};

struct ProductionValidationIssue {
    ProductionValidationIssueType type{
        ProductionValidationIssueType::UnknownId
    };
    std::size_t sheetIndex{};
    std::string instanceId;
    std::string relatedInstanceId;
    std::string unitId;
    double measuredMm{};
    double requiredMm{};
    std::string message;
};

struct AdaptiveRepairChange {
    std::size_t sheetIndex{};
    std::size_t afterSheetIndex{};
    Placement before{};
    Placement after{};
    bool conflictGroup{};
    bool extracted{};
    bool moved{};
    bool stationary{};
};

struct AdaptiveRepairRound {
    std::size_t roundIndex{};
    std::size_t repairedLevel{};
    std::size_t validationSequence{};
    std::vector<std::vector<Placement>> beforeSheets;
    std::vector<std::vector<Placement>> afterSheets;
    std::vector<AdaptiveRepairChange> changes;
    std::vector<std::string> conflictIds;
    std::vector<std::string> extractedIds;
    std::vector<std::string> movedIds;
    std::vector<std::string> stationaryIds;
    std::vector<std::vector<std::string>> conflictLevels;
    std::size_t collisionCountAfter{};
    std::size_t gapViolationCountAfter{};
    std::size_t marginViolationCountAfter{};
    bool validAfter{false};
};

struct ProductionValidationReport {
    bool valid{true};
    bool pipelineValid{false};
    ProductionPipelineStage pipelineStage{ProductionPipelineStage::Nesting};
    std::string pipelineMessage;
    std::size_t checkedPlacements{};
    std::size_t collisionCount{};
    std::size_t gapViolationCount{};
    std::size_t marginViolationCount{};
    std::size_t duplicateIdCount{};
    std::size_t missingIdCount{};
    std::size_t unknownIdCount{};
    std::size_t repairAttempts{};
    std::size_t adaptiveRepairRounds{};
    std::size_t adaptiveRepairGroupSize{};
    std::uint64_t repairElapsedMs{};
    bool repaired{};
    bool coverageComplete{false};
    std::size_t expectedInstanceCount{};
    std::size_t placedInstanceCount{};
    std::size_t unplacedInstanceCount{};
    std::vector<AdaptiveRepairChange> adaptiveChanges;
    std::vector<AdaptiveRepairRound> adaptiveHistory;
    std::vector<std::string> adaptiveConflictIds;
    std::vector<std::string> adaptiveExtractedIds;
    std::vector<std::string> adaptiveMovedIds;
    std::vector<std::string> adaptiveStationaryIds;
    std::vector<ProductionValidationIssue> issues;
};

ProductionValidationReport validateProductionResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    const Result& result
);

// Attempts to repair an invalid result by running several bounded,
// higher-quality nesting passes. The current result is replaced only when
// the repaired candidate passes Production Validator.
bool repairProductionResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    Result& result,
    ProductionValidationReport* reportOut = nullptr
);

const char* productionValidationIssueTypeName(
    ProductionValidationIssueType type
);

} // namespace sheetnest
