#include "sheetnest/nfp.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_set>
#include <utility>
#include <vector>

namespace sheetnest::nfp {
namespace {

constexpr double kEps = 1e-9;

double cross(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) -
           (b.y - a.y) * (c.x - a.x);
}

bool samePoint(Point a, Point b) {
    return std::hypot(a.x - b.x, a.y - b.y) <= 1e-8;
}

Polygon cleanPolygon(const Polygon& input) {
    Polygon p;
    for (const auto& point : input) {
        if (p.empty() || !samePoint(p.back(), point)) {
            p.push_back(point);
        }
    }
    if (p.size() > 1 && samePoint(p.front(), p.back())) {
        p.pop_back();
    }
    return p;
}

double signedArea(const Polygon& p) {
    if (p.size() < 3) return 0.0;
    double area = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto& a = p[i];
        const auto& b = p[(i + 1) % p.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return area * 0.5;
}

bool pointInTriangle(Point p, Point a, Point b, Point c) {
    const double c1 = cross(a, b, p);
    const double c2 = cross(b, c, p);
    const double c3 = cross(c, a, p);
    const bool hasNeg = c1 < -kEps || c2 < -kEps || c3 < -kEps;
    const bool hasPos = c1 > kEps || c2 > kEps || c3 > kEps;
    return !(hasNeg && hasPos);
}

Polygon convexHull(std::vector<Point> points) {
    if (points.size() <= 1) return points;

    std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    });

    points.erase(
        std::unique(points.begin(), points.end(), [](const Point& a, const Point& b) {
            return samePoint(a, b);
        }),
        points.end()
    );

    if (points.size() <= 2) return points;

    Polygon lower;
    for (const auto& p : points) {
        while (lower.size() >= 2 &&
               cross(lower[lower.size() - 2], lower.back(), p) <= kEps) {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    Polygon upper;
    for (std::size_t i = points.size(); i-- > 0;) {
        const auto& p = points[i];
        while (upper.size() >= 2 &&
               cross(upper[upper.size() - 2], upper.back(), p) <= kEps) {
            upper.pop_back();
        }
        upper.push_back(p);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

Polygon reflected(const Polygon& polygon) {
    Polygon result;
    result.reserve(polygon.size());
    for (const auto& p : polygon) {
        result.push_back({-p.x, -p.y});
    }
    return result;
}

} // namespace

std::vector<Polygon> convexDecompose(const Polygon& input) {
    Polygon polygon = cleanPolygon(input);
    if (polygon.size() < 3) return {};

    if (signedArea(polygon) < 0.0) {
        std::reverse(polygon.begin(), polygon.end());
    }

    std::vector<std::size_t> indices(polygon.size());
    for (std::size_t i = 0; i < polygon.size(); ++i) indices[i] = i;

    std::vector<Polygon> triangles;
    triangles.reserve(polygon.size() - 2);

    std::size_t guard = 0;
    while (indices.size() > 3 && guard++ < polygon.size() * polygon.size()) {
        bool clipped = false;

        for (std::size_t i = 0; i < indices.size(); ++i) {
            const std::size_t ia = indices[(i + indices.size() - 1) % indices.size()];
            const std::size_t ib = indices[i];
            const std::size_t ic = indices[(i + 1) % indices.size()];

            const Point a = polygon[ia];
            const Point b = polygon[ib];
            const Point c = polygon[ic];

            if (cross(a, b, c) <= kEps) continue;

            bool containsOther = false;
            for (const auto idx : indices) {
                if (idx == ia || idx == ib || idx == ic) continue;
                if (pointInTriangle(polygon[idx], a, b, c)) {
                    containsOther = true;
                    break;
                }
            }

            if (containsOther) continue;

            triangles.push_back({a, b, c});
            indices.erase(indices.begin() + static_cast<std::ptrdiff_t>(i));
            clipped = true;
            break;
        }

        if (!clipped) {
            return {};
        }
    }

    if (indices.size() == 3) {
        triangles.push_back({
            polygon[indices[0]],
            polygon[indices[1]],
            polygon[indices[2]]
        });
    }

    return triangles;
}

Polygon minkowskiConvexSum(
    const Polygon& a,
    const Polygon& reflectedB
) {
    if (a.size() < 3 || reflectedB.size() < 3) return {};

    std::vector<Point> sums;
    sums.reserve(a.size() * reflectedB.size());

    for (const auto& p : a) {
        for (const auto& q : reflectedB) {
            sums.push_back({p.x + q.x, p.y + q.y});
        }
    }

    return convexHull(std::move(sums));
}

std::vector<Polygon> noFitPolygons(
    const Polygon& fixed,
    const Polygon& moving
) {
    const auto fixedPieces = convexDecompose(fixed);
    const auto movingPieces = convexDecompose(moving);

    std::vector<Polygon> result;
    for (const auto& a : fixedPieces) {
        for (const auto& b : movingPieces) {
            const auto nfp = minkowskiConvexSum(a, reflected(b));
            if (nfp.size() >= 3) {
                result.push_back(nfp);
            }
        }
    }
    return result;
}

std::vector<Point> noFitVertices(
    const Polygon& fixed,
    const Polygon& moving
) {
    std::vector<Point> vertices;

    const auto polygons = noFitPolygons(fixed, moving);
    for (const auto& polygon : polygons) {
        vertices.insert(vertices.end(), polygon.begin(), polygon.end());
    }

    std::sort(vertices.begin(), vertices.end(), [](const Point& a, const Point& b) {
        if (std::abs(a.x - b.x) > 1e-8) return a.x < b.x;
        return a.y < b.y;
    });

    vertices.erase(
        std::unique(vertices.begin(), vertices.end(), [](const Point& a, const Point& b) {
            return samePoint(a, b);
        }),
        vertices.end()
    );

    return vertices;
}

} // namespace sheetnest::nfp
