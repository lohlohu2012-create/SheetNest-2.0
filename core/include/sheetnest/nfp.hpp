#pragma once

#include "geometry.hpp"

#include <vector>

namespace sheetnest::nfp {

std::vector<Polygon> convexDecompose(const Polygon& polygon);

Polygon minkowskiConvexSum(
    const Polygon& a,
    const Polygon& reflectedB
);

std::vector<Polygon> noFitPolygons(
    const Polygon& fixed,
    const Polygon& moving
);

std::vector<Point> noFitVertices(
    const Polygon& fixed,
    const Polygon& moving
);

} // namespace sheetnest::nfp
