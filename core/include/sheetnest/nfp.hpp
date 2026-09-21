#pragma once

#include "geometry.hpp"

#include <cstddef>
#include <vector>

namespace sheetnest::nfp {

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

std::vector<Polygon> convexDecompose(const Polygon& polygon);

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
    double clearanceMm = 0.0
);

std::vector<Point> noFitVertices(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation = 0,
    double clearanceMm = 0.0
);

FeasibilityRegion feasibilityRegion(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation,
    double minX,
    double minY,
    double maxX,
    double maxY,
    double clearanceMm = 0.0
);

std::vector<Point> pointsOnFeasibilityBoundary(
    const FeasibilityRegion& region,
    double spacingMm
);

void clearCache();
CacheStats cacheStats();

} // namespace sheetnest::nfp
