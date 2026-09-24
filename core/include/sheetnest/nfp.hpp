#pragma once

#include "geometry.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace sheetnest::nfp {

struct PolygonWithHoles {
    Polygon outer;
    std::vector<Polygon> holes;
};

struct PolygonRegion {
    std::vector<PolygonWithHoles> components;
};

PolygonWithHoles normalizePolygonWithHoles(const Polygon& outer,
                                           const std::vector<Polygon>& holes);
bool validatePolygonWithHoles(const PolygonWithHoles& region);
bool pointInPolygonWithHoles(const Point& p, const PolygonWithHoles& region);
std::vector<PolygonWithHoles> classifyPolygonLoops(
    const std::vector<Polygon>& loops);

struct NfpRunControl {
    std::function<bool()> shouldStop;
    std::size_t maxInputVertices{512};
    std::size_t maxConvexPieces{128};
    std::size_t maxPairwisePolygons{4096};
    std::size_t maxUnionSegments{20000};
    std::size_t* timeoutCount{};
    std::size_t* complexityFallbackCount{};
    std::size_t* cacheHitCount{};
    std::size_t* cacheMissCount{};
    mutable bool timeoutRecorded{false};

    bool stop() const {
        if (!shouldStop || !shouldStop()) return false;
        if (!timeoutRecorded) {
            timeoutRecorded = true;
            if (timeoutCount) ++(*timeoutCount);
        }
        return true;
    }

    void complexityFallback() const {
        if (complexityFallbackCount) ++(*complexityFallbackCount);
    }
};

struct CacheStats {
    std::size_t hits{};
    std::size_t misses{};
    std::size_t entries{};
};

struct FeasibilitySegment {
    Point a{};
    Point b{};
};

struct FeasibilityRegion {
    std::vector<FeasibilitySegment> boundary;
    std::vector<FeasibilitySegment> sheetBoundary;
};

struct NfpValidationReport {
    bool valid{true};
    std::size_t loops{};
    std::size_t holes{};
    std::size_t degenerateLoops{};
    std::size_t selfIntersectingLoops{};
    std::size_t nonFiniteVertices{};
    std::size_t openBoundarySegments{};
    std::size_t intersectingLoops{};
    std::size_t invalidTopologyLoops{};
    std::size_t invalidOrientationLoops{};
    std::size_t duplicateLoops{};
};

NfpValidationReport validateNfp(
    const std::vector<Polygon>& polygons
);

std::vector<Polygon> convexDecompose(const Polygon& polygon, const NfpRunControl* control = nullptr);

Polygon minkowskiConvexSum(
    const Polygon& a,
    const Polygon& reflectedB
);

// Returns the exact union boundary of pairwise Minkowski sums of the
// convex decomposition components, i.e. the union-form NFP for simple
// non-self-intersecting polygons.
std::vector<Polygon> noFitPolygons(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation = 0,
    double clearanceMm = 0.0,
    const NfpRunControl* control = nullptr
);

std::vector<Point> noFitVertices(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation = 0,
    double clearanceMm = 0.0,
    const NfpRunControl* control = nullptr
);

FeasibilityRegion feasibilityRegion(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation,
    double minX,
    double minY,
    double maxX,
    double maxY,
    double clearanceMm = 0.0,
    const NfpRunControl* control = nullptr
);

std::vector<Point> pointsOnFeasibilityBoundary(
    const FeasibilityRegion& region,
    double spacingMm,
    std::size_t maxPoints = 256,
    bool includeSheetBoundary = true
);


// Bounded candidate search over the continuous NFP feasibility boundary.
// The search never assumes that NFP vertices are sufficient: boundary
// segments are sampled continuously and the optional exact validator remains
// authoritative for technological clearance and true-shape collision rules.
struct SearchOptions {
    double boundarySpacingMm{5.0};
    std::size_t maxCandidates{256};
    bool includeSheetBoundary{true};
    bool includeNfpVertices{true};
    bool includeBoundaryMidpoints{true};
    std::function<bool(const Point&)> isFeasible;
};

struct SearchTelemetry {
    std::size_t generated{};
    std::size_t deduplicated{};
    std::size_t exactChecks{};
    std::size_t feasible{};
    std::size_t rejected{};
    std::size_t boundarySegments{};
    std::size_t boundarySamples{};
    std::size_t nfpVertices{};
    bool budgetExceeded{};
    bool stopped{};
};

struct SearchResult {
    std::vector<Point> points;
    SearchTelemetry telemetry;
};

SearchResult searchFeasibleBoundary(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation,
    double minX,
    double minY,
    double maxX,
    double maxY,
    double clearanceMm,
    const SearchOptions& options = {},
    const NfpRunControl* control = nullptr
);

void clearCache();
CacheStats cacheStats();

} // namespace sheetnest::nfp
