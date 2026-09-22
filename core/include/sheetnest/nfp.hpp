#pragma once

#include "geometry.hpp"

#include <cstddef>
#include <functional>
#include <vector>

namespace sheetnest::nfp {

struct NfpRunControl {
    std::function<bool()> shouldStop;
    std::size_t maxInputVertices{512};
    std::size_t maxConvexPieces{128};
    std::size_t maxPairwisePolygons{4096};
    std::size_t maxUnionSegments{20000};
    std::size_t* timeoutCount{};
    std::size_t* complexityFallbackCount{};
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

void clearCache();
CacheStats cacheStats();

} // namespace sheetnest::nfp
