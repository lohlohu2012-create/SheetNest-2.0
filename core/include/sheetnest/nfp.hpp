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

void clearCache();
CacheStats cacheStats();

} // namespace sheetnest::nfp
