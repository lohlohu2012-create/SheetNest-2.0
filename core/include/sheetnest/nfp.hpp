#pragma once
#include "geometry.hpp"

namespace sheetnest {

enum class NfpQuality {
  ExactConvex,
  ConservativeConvexHull
};

struct NfpResult {
  Polygon boundary;
  NfpQuality quality{NfpQuality::ConservativeConvexHull};
  bool valid{};
};

bool isConvex(const Polygon&);
Polygon convexHull(const Polygon&);

// Forbidden translation region for a moving polygon against a fixed polygon.
// For two convex polygons this is the exact Minkowski sum fixed + (-moving).
// For non-convex input the returned polygon is a conservative convex-hull
// candidate region and must not be treated as an exact collision boundary.
NfpResult buildNfp(const Polygon& fixed, const Polygon& moving);

}
