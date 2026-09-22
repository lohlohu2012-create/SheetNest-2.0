#pragma once

#include "nesting.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sheetnest {

enum class ProductionValidationIssueType {
    Collision,
    Gap,
    Margin,
    DuplicateId,
    MissingId
};

struct ProductionValidationIssue {
    ProductionValidationIssueType type{
        ProductionValidationIssueType::MissingId
    };
    std::size_t sheetIndex{};
    std::string instanceId;
    std::string relatedInstanceId;
    double measured{};
    double required{};
    std::string message;
};

struct ProductionValidationReport {
    bool valid{true};
    std::size_t collisionCount{};
    std::size_t gapCount{};
    std::size_t marginCount{};
    std::size_t duplicateIdCount{};
    std::size_t missingIdCount{};
    std::vector<ProductionValidationIssue> issues;

    bool hasErrors() const {
        return !issues.empty();
    }
};

const char* productionValidationIssueTypeName(
    ProductionValidationIssueType type
);

ProductionValidationReport validateProductionResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    const Result& result
);

} // namespace sheetnest
