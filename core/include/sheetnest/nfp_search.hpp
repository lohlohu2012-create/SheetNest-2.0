#pragma once

#include "nfp.hpp"

#include <cstddef>
#include <vector>

namespace sheetnest::nfp {

struct SearchOptions {
    double budgetMs{250.0};
    double segmentSpacingMm{8.0};
    std::size_t maxCandidates{256};
    bool includeSheetBoundary{true};
};

struct SearchTelemetry {
    std::size_t candidateChecks{};
    std::size_t segmentsSampled{};
    std::size_t boundaryCandidates{};
    std::size_t sheetBoundaryCandidates{};
    std::size_t budgetExceededCount{};
    double elapsedMs{};
    bool budgetExceeded{};
};

struct SearchResult {
    std::vector<Point> candidates;
    SearchTelemetry telemetry;
};

SearchResult searchFeasibilityRegion(
    const FeasibilityRegion& region,
    const SearchOptions& options = {}
);

} // namespace sheetnest::nfp
