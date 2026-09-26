#include "sheetnest/runtime_tuning.hpp"
#include <algorithm>
#include <cstdint>
namespace sheetnest {
RuntimeTuningReport tuneNestingOptions(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& baseOptions
) {
    RuntimeTuningReport report;
    report.options = baseOptions;
    report.instanceCount = instances.size();
    for (const auto& instance : instances) {
        report.totalVertices += instance.part.outer.size();
        report.maxPartVertices = std::max(report.maxPartVertices, instance.part.outer.size());
        for (const auto& hole : instance.part.holes) {
            report.totalVertices += hole.size();
            report.maxPartVertices = std::max(report.maxPartVertices, hole.size());
        }
    }
    if (!baseOptions.enableAdaptiveRuntimeTuning) {
        report.profile = "Manual";
        report.reason = "Adaptive runtime tuning disabled";
        return report;
    }
    const double averageVertices = instances.empty() ? 0.0 :
        static_cast<double>(report.totalVertices) / static_cast<double>(instances.size());
    const bool complexGeometry =
        static_cast<double>(report.maxPartVertices) >= baseOptions.nfpComplexityVertexThreshold ||
        averageVertices >= baseOptions.nfpComplexityVertexThreshold * 0.55;
    const bool massNesting =
        static_cast<double>(instances.size()) >= baseOptions.nfpComplexityPartCountThreshold;
    if (!complexGeometry && !massNesting) {
        report.profile = "Balanced";
        report.reason = "Geometry is below adaptive complexity thresholds";
        return report;
    }
    report.tuned = true;
    if (complexGeometry) {
        report.profile = massNesting ? "Mass-Complex" : "Complex-Geometry";
        report.options.nfpMaxInputVertices = std::min(report.options.nfpMaxInputVertices, static_cast<std::size_t>(384));
        report.options.nfpMaxConvexPieces = std::min(report.options.nfpMaxConvexPieces, static_cast<std::size_t>(96));
        report.options.nfpMaxPairwisePolygons = std::min(report.options.nfpMaxPairwisePolygons, static_cast<std::size_t>(2048));
        report.options.nfpMaxUnionSegments = std::min(report.options.nfpMaxUnionSegments, static_cast<std::size_t>(12000));
        report.options.nfpCandidateBudget = std::min(report.options.nfpCandidateBudget, report.options.nfpCandidateBudgetComplex);
        report.options.nfpTimeBudgetMs = std::min<std::uint64_t>(report.options.nfpTimeBudgetMs, 180);
        report.options.candidateVariantBudget = std::min(report.options.candidateVariantBudget, static_cast<std::size_t>(6));
        report.options.smallPartCandidateBudget = std::min(report.options.smallPartCandidateBudget, static_cast<std::size_t>(768));
        report.options.residualRetryPasses = std::min(report.options.residualRetryPasses, static_cast<std::size_t>(2));
        report.reason = massNesting ? "Mass nesting with complex geometry: bounded NFP breadth" : "Complex geometry: bounded NFP breadth";
    } else {
        report.profile = "Mass-Nesting";
        report.options.nfpCandidateBudget = std::min(report.options.nfpCandidateBudget, static_cast<std::size_t>(384));
        report.options.nfpTimeBudgetMs = std::min<std::uint64_t>(report.options.nfpTimeBudgetMs, 220);
        report.options.candidateVariantBudget = std::min(report.options.candidateVariantBudget, static_cast<std::size_t>(6));
        report.options.smallPartCandidateBudget = std::min(report.options.smallPartCandidateBudget, static_cast<std::size_t>(1024));
        report.options.residualRetryPasses = std::min(report.options.residualRetryPasses, static_cast<std::size_t>(2));
        report.reason = "Mass nesting: bounded candidate breadth";
    }
    if (sheet.width <= 0.0 || sheet.height <= 0.0)
        report.options.nfpCandidateBudget = std::min(report.options.nfpCandidateBudget, static_cast<std::size_t>(128));
    return report;
}
} // namespace sheetnest
