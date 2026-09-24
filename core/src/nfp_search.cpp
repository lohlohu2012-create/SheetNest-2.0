#include "sheetnest/nfp_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_set>
#include <string>

namespace sheetnest::nfp {
namespace {

std::string pointKey(Point p) {
    const auto q = [](double v) {
        return std::to_string(static_cast<long long>(std::llround(v * 1000000.0)));
    };
    return q(p.x) + "," + q(p.y);
}

bool budgetHit(
    const std::chrono::steady_clock::time_point& started,
    double budgetMs
) {
    if (budgetMs <= 0.0) return false;
    const auto elapsed =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started
        ).count();
    return elapsed >= budgetMs;
}

} // namespace

SearchResult searchFeasibilityRegion(
    const FeasibilityRegion& region,
    const SearchOptions& options
) {
    SearchResult result;
    const auto started = std::chrono::steady_clock::now();

    const std::size_t maxCandidates =
        std::max<std::size_t>(1, options.maxCandidates);
    const double spacing =
        std::max(0.1, options.segmentSpacingMm);

    std::unordered_set<std::string> seen;
    seen.reserve(maxCandidates * 2);

    auto collect = [&](bool sheetBoundary) {
        if (result.candidates.size() >= maxCandidates) return;

        const auto& boundary =
            sheetBoundary ? region.sheetBoundary : region.boundary;

        if (boundary.empty()) return;

        ++result.telemetry.segmentsSampled;

        const auto points = pointsOnFeasibilityBoundary(
            FeasibilityRegion{
                sheetBoundary ? std::vector<FeasibilitySegment>{}
                              : boundary,
                sheetBoundary ? boundary : std::vector<FeasibilitySegment>{}
            },
            spacing,
            maxCandidates - result.candidates.size(),
            true
        );

        for (const auto& point : points) {
            if (budgetHit(started, options.budgetMs)) {
                result.telemetry.budgetExceeded = true;
                ++result.telemetry.budgetExceededCount;
                return;
            }

            ++result.telemetry.candidateChecks;
            if (seen.insert(pointKey(point)).second) {
                result.candidates.push_back(point);
                if (sheetBoundary) {
                    ++result.telemetry.sheetBoundaryCandidates;
                } else {
                    ++result.telemetry.boundaryCandidates;
                }
            }

            if (result.candidates.size() >= maxCandidates) return;
        }
    };

    collect(false);
    if (!result.telemetry.budgetExceeded &&
        options.includeSheetBoundary) {
        collect(true);
    }

    std::sort(
        result.candidates.begin(),
        result.candidates.end(),
        [](Point a, Point b) {
            if (std::abs(a.y - b.y) > 1e-9) return a.y < b.y;
            return a.x < b.x;
        }
    );

    result.telemetry.elapsedMs =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started
        ).count();

    return result;
}

} // namespace sheetnest::nfp
