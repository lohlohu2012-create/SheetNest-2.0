#include "sheetnest/nfp.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sheetnest::nfp {
namespace {

constexpr double kEps = 1e-9;
constexpr double kPointEps = 1e-8;
constexpr double kQuant = 1e6;
constexpr std::size_t kMaxCacheEntries = 4096;

struct SegmentPiece {
    Point a{};
    Point b{};
};

struct ClusteredSegment {
    std::size_t a{};
    std::size_t b{};
};

struct CacheData {
    std::vector<Polygon> polygons;
};

struct CacheStore {
    std::mutex mutex;
    std::unordered_map<std::string, CacheData> entries;
    std::size_t hits{};
    std::size_t misses{};
};

CacheStore& cacheStore() {
    static CacheStore store;
    return store;
}

double cross(Point a, Point b, Point c) {
    return (b.x - a.x) * (c.y - a.y) -
           (b.y - a.y) * (c.x - a.x);
}

double crossVec(Point a, Point b) {
    return a.x * b.y - a.y * b.x;
}

double dot(Point a, Point b) {
    return a.x * b.x + a.y * b.y;
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

bool samePoint(Point a, Point b, double tolerance = kPointEps) {
    return std::hypot(a.x - b.x, a.y - b.y) <= tolerance;
}

Polygon cleanPolygon(const Polygon& input) {
    Polygon p;
    p.reserve(input.size());

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

bool pointOnSegment(Point p, Point a, Point b, double tolerance = kPointEps) {
    if (std::abs(cross(a, b, p)) >
        tolerance * std::max(1.0, std::hypot(b.x - a.x, b.y - a.y))) {
        return false;
    }

    return p.x >= std::min(a.x, b.x) - tolerance &&
           p.x <= std::max(a.x, b.x) + tolerance &&
           p.y >= std::min(a.y, b.y) - tolerance &&
           p.y <= std::max(a.y, b.y) + tolerance;
}

bool pointInPolygonInclusive(
    Point p,
    const Polygon& polygon
) {
    if (polygon.size() < 3) return false;

    bool inside = false;
    for (std::size_t i = 0, j = polygon.size() - 1;
         i < polygon.size();
         j = i++) {
        const Point a = polygon[i];
        const Point b = polygon[j];

        if (pointOnSegment(p, a, b)) return true;

        if ((a.y > p.y) != (b.y > p.y)) {
            const double x =
                (b.x - a.x) * (p.y - a.y) /
                    ((b.y - a.y) + std::numeric_limits<double>::epsilon()) +
                a.x;
            if (p.x < x) inside = !inside;
        }
    }

    return inside;
}

Polygon convexHull(std::vector<Point> points) {
    if (points.size() <= 1) return points;

    std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
        if (std::abs(a.x - b.x) > kPointEps) return a.x < b.x;
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
    for (const auto& point : points) {
        while (lower.size() >= 2 &&
               cross(lower[lower.size() - 2], lower.back(), point) <= kEps) {
            lower.pop_back();
        }
        lower.push_back(point);
    }

    Polygon upper;
    for (std::size_t i = points.size(); i-- > 0;) {
        const auto& point = points[i];
        while (upper.size() >= 2 &&
               cross(upper[upper.size() - 2], upper.back(), point) <= kEps) {
            upper.pop_back();
        }
        upper.push_back(point);
    }

    lower.pop_back();
    upper.pop_back();
    lower.insert(lower.end(), upper.begin(), upper.end());
    return lower;
}

bool pointInTriangle(Point p, Point a, Point b, Point c) {
    const double c1 = cross(a, b, p);
    const double c2 = cross(b, c, p);
    const double c3 = cross(c, a, p);

    const bool hasNeg = c1 < -kEps || c2 < -kEps || c3 < -kEps;
    const bool hasPos = c1 > kEps || c2 > kEps || c3 > kEps;
    return !(hasNeg && hasPos);
}

Polygon reflected(const Polygon& polygon) {
    Polygon result;
    result.reserve(polygon.size());
    for (const auto& point : polygon) {
        result.push_back({-point.x, -point.y});
    }
    return result;
}

std::pair<Polygon, Point> canonicalize(const Polygon& polygon) {
    if (polygon.empty()) return {{}, {}};

    Polygon cleaned = cleanPolygon(polygon);
    if (cleaned.empty()) return {{}, {}};

    const auto b = bounds(cleaned);
    for (auto& point : cleaned) {
        point.x -= b.minX;
        point.y -= b.minY;
    }

    // Translation invariance alone is not enough for a cache key: DXF and
    // boolean geometry code may expose the same ring with a different start
    // vertex. Canonicalize the cyclic start without changing orientation.
    std::size_t best = 0;
    for (std::size_t i = 1; i < cleaned.size(); ++i) {
        const auto& a = cleaned[i];
        const auto& z = cleaned[best];
        if (a.x < z.x - kPointEps ||
            (std::abs(a.x - z.x) <= kPointEps && a.y < z.y - kPointEps)) {
            best = i;
        }
    }

    Polygon normalized;
    normalized.reserve(cleaned.size());
    for (std::size_t offset = 0; offset < cleaned.size(); ++offset) {
        normalized.push_back(cleaned[(best + offset) % cleaned.size()]);
    }

    return {std::move(normalized), {b.minX, b.minY}};
}

bool validCachedNfp(const std::vector<Polygon>& polygons) {
    if (polygons.empty()) return false;

    for (const auto& polygon : polygons) {
        if (polygon.size() < 3) return false;

        double area = 0.0;
        for (const auto& point : polygon) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                return false;
            }
        }
        area = signedArea(polygon);
        if (!std::isfinite(area) || std::abs(area) <= kEps) {
            return false;
        }
    }
    return true;
}

long long quantize(double value) {
    return static_cast<long long>(std::llround(value * kQuant));
}

void appendKeyPolygon(std::ostringstream& out, const Polygon& polygon) {
    out << polygon.size() << ';';
    for (const auto& point : polygon) {
        out << quantize(point.x) << ','
            << quantize(point.y) << ';';
    }
}

std::string makeCacheKey(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation,
    double clearanceMm
) {
    const auto [fixedCanonical, fixedOrigin] = canonicalize(fixed);
    const auto [movingCanonical, movingOrigin] = canonicalize(
        rotate(moving, rotation)
    );

    (void)fixedOrigin;
    (void)movingOrigin;

    std::ostringstream key;
    key << "v2|rot=" << rotation
        << "|gap=" << quantize(clearanceMm)
        << "|F=";
    appendKeyPolygon(key, fixedCanonical);
    key << "|M=";
    appendKeyPolygon(key, movingCanonical);
    return key.str();
}

std::vector<double> splitParameters(
    Point a,
    Point b,
    const std::vector<Polygon>& others
) {
    std::vector<double> parameters{0.0, 1.0};

    const Point r{b.x - a.x, b.y - a.y};
    const double rr = dot(r, r);
    if (rr <= kEps) return parameters;

    auto addParameter = [&](Point p) {
        const double t = dot(
            {p.x - a.x, p.y - a.y},
            r
        ) / rr;
        if (t > kEps && t < 1.0 - kEps) {
            parameters.push_back(std::clamp(t, 0.0, 1.0));
        }
    };

    for (const auto& polygon : others) {
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            const Point c = polygon[i];
            const Point d = polygon[(i + 1) % polygon.size()];
            const Point s{d.x - c.x, d.y - c.y};

            const double denominator = crossVec(r, s);
            const Point ca{c.x - a.x, c.y - a.y};

            if (std::abs(denominator) > kEps) {
                const double t = crossVec(ca, s) / denominator;
                const double u = crossVec(ca, r) / denominator;
                if (t > -kEps && t < 1.0 + kEps &&
                    u > -kEps && u < 1.0 + kEps) {
                    parameters.push_back(std::clamp(t, 0.0, 1.0));
                }
            } else if (std::abs(crossVec(ca, r)) <= kEps) {
                // Collinear overlap: split at all overlap endpoints.
                addParameter(c);
                addParameter(d);
            }
        }
    }

    std::sort(parameters.begin(), parameters.end());
    parameters.erase(
        std::unique(parameters.begin(), parameters.end(), [](double a, double b) {
            return std::abs(a - b) <= 1e-10;
        }),
        parameters.end()
    );

    return parameters;
}

Point outwardNormal(
    Point a,
    Point b,
    double polygonArea
) {
    const Point edge{b.x - a.x, b.y - a.y};
    const double length = std::hypot(edge.x, edge.y);
    if (length <= kEps) return {};

    if (polygonArea >= 0.0) {
        return {edge.y / length, -edge.x / length};
    }
    return {-edge.y / length, edge.x / length};
}

std::vector<SegmentPiece> unionBoundaryPieces(
    const std::vector<Polygon>& polygons,
    const NfpRunControl* control
) {
    std::vector<SegmentPiece> retained;
    if (polygons.empty()) return retained;

    std::unordered_map<std::string, bool> seen;

    auto endpointKey = [](Point p) {
        return std::to_string(quantize(p.x)) + "," +
               std::to_string(quantize(p.y));
    };

    auto undirectedKey = [&](Point a, Point b) {
        std::string ka = endpointKey(a);
        std::string kb = endpointKey(b);
        if (kb < ka) std::swap(ka, kb);
        return ka + "|" + kb;
    };

    for (std::size_t polyIndex = 0; polyIndex < polygons.size(); ++polyIndex) {
        if (control && control->stop()) return retained;
        const Polygon polygon = cleanPolygon(polygons[polyIndex]);
        if (polygon.size() < 3) continue;

        const double area = signedArea(polygon);
        if (std::abs(area) <= kEps) continue;

        for (std::size_t edgeIndex = 0; edgeIndex < polygon.size(); ++edgeIndex) {
            if (control && control->stop()) return retained;
            const Point a = polygon[edgeIndex];
            const Point b = polygon[(edgeIndex + 1) % polygon.size()];
            const Point edge{b.x - a.x, b.y - a.y};
            const double edgeLength = std::hypot(edge.x, edge.y);
            if (edgeLength <= kPointEps) continue;

            std::vector<Polygon> others;
            others.reserve(polygons.size() - 1);
            for (std::size_t j = 0; j < polygons.size(); ++j) {
                if (j != polyIndex) others.push_back(polygons[j]);
            }

            const auto parameters = splitParameters(a, b, others);
            const Point outward = outwardNormal(a, b, area);

            for (std::size_t p = 1; p < parameters.size(); ++p) {
                if (control && control->stop()) return retained;
                const double t0 = parameters[p - 1];
                const double t1 = parameters[p];
                if (t1 - t0 <= 1e-10) continue;

                const Point p0{
                    a.x + edge.x * t0,
                    a.y + edge.y * t0
                };
                const Point p1{
                    a.x + edge.x * t1,
                    a.y + edge.y * t1
                };
                const Point mid{
                    (p0.x + p1.x) * 0.5,
                    (p0.y + p1.y) * 0.5
                };

                bool strictlyInsideOther = false;
                bool onBoundaryOther = false;

                for (const auto& other : others) {
                    if (control && control->stop()) return retained;
                    if (pointInPolygonInclusive(mid, other)) {
                        onBoundaryOther = false;
                        strictlyInsideOther = true;

                        // If midpoint lies on an edge, determine whether the
                        // outside side of the current polygon is also occupied.
                        bool onEdge = false;
                        for (std::size_t e = 0; e < other.size(); ++e) {
                            if (pointOnSegment(
                                    mid,
                                    other[e],
                                    other[(e + 1) % other.size()])) {
                                onEdge = true;
                                break;
                            }
                        }
                        if (onEdge) {
                            strictlyInsideOther = false;
                            onBoundaryOther = true;
                        }

                        if (strictlyInsideOther) break;
                    }
                }

                bool keep = !strictlyInsideOther;

                if (onBoundaryOther) {
                    const double delta =
                        std::max(1e-7, std::min(1e-4, edgeLength * 1e-4));
                    const Point outside{
                        mid.x + outward.x * delta,
                        mid.y + outward.y * delta
                    };

                    bool outsideOccupied = false;
                    for (const auto& other : others) {
                        if (pointInPolygonInclusive(outside, other)) {
                            outsideOccupied = true;
                            break;
                        }
                    }
                    keep = !outsideOccupied;
                }

                if (!keep) continue;

                const std::string key = undirectedKey(p0, p1);
                if (seen.emplace(key, true).second) {
                    retained.push_back({p0, p1});
                }
            }
        }
    }

    if (control && retained.size() > control->maxUnionSegments) {
        control->complexityFallback();
        retained.clear();
    }
    return retained;
}

std::vector<Polygon> assembleBoundaryLoops(
    const std::vector<SegmentPiece>& pieces
) {
    if (pieces.empty()) return {};

    struct Node {
        Point point{};
    };

    auto nodeKey = [](Point p) {
        return std::to_string(quantize(p.x)) + "," +
               std::to_string(quantize(p.y));
    };

    std::vector<Node> nodes;
    std::unordered_map<std::string, std::size_t> nodeMap;
    std::vector<ClusteredSegment> segments;

    auto nodeIndex = [&](Point p) {
        const std::string key = nodeKey(p);
        const auto it = nodeMap.find(key);
        if (it != nodeMap.end()) return it->second;

        const auto index = nodes.size();
        nodes.push_back({p});
        nodeMap.emplace(key, index);
        return index;
    };

    for (const auto& piece : pieces) {
        const std::size_t a = nodeIndex(piece.a);
        const std::size_t b = nodeIndex(piece.b);
        if (a != b) segments.push_back({a, b});
    }

    std::vector<std::vector<std::size_t>> outgoing(nodes.size());
    for (std::size_t i = 0; i < segments.size(); ++i) {
        outgoing[segments[i].a].push_back(i);
    }

    std::vector<bool> used(segments.size(), false);
    std::vector<Polygon> loops;

    for (std::size_t start = 0; start < segments.size(); ++start) {
        if (used[start]) continue;

        const std::size_t startNode = segments[start].a;
        std::size_t currentSegment = start;
        std::vector<Point> loop;

        for (std::size_t guard = 0; guard <= segments.size(); ++guard) {
            if (used[currentSegment]) break;
            used[currentSegment] = true;

            const auto& segment = segments[currentSegment];
            const Point a = nodes[segment.a].point;
            const Point b = nodes[segment.b].point;

            if (loop.empty()) loop.push_back(a);
            loop.push_back(b);

            if (segment.b == startNode) {
                break;
            }

            const Point incoming{
                b.x - a.x,
                b.y - a.y
            };

            std::size_t best = segments.size();
            double bestTurn = -std::numeric_limits<double>::infinity();

            for (const auto candidate : outgoing[segment.b]) {
                if (used[candidate]) continue;

                const Point ca = nodes[segments[candidate].a].point;
                const Point cb = nodes[segments[candidate].b].point;
                const Point outgoingVector{
                    cb.x - ca.x,
                    cb.y - ca.y
                };

                double angle = std::atan2(
                    crossVec(incoming, outgoingVector),
                    dot(incoming, outgoingVector)
                );

                if (angle < 0.0) angle += 2.0 * 3.14159265358979323846;

                if (angle > bestTurn) {
                    bestTurn = angle;
                    best = candidate;
                }
            }

            if (best == segments.size()) break;
            currentSegment = best;
        }

        if (loop.size() >= 4 &&
            samePoint(loop.front(), loop.back())) {
            loop.pop_back();
            const double area = signedArea(loop);
            if (std::abs(area) > 1e-8) {
                loops.push_back(std::move(loop));
            }
        }
    }

    return loops;
}

std::vector<Polygon> unionPolygons(
    const std::vector<Polygon>& polygons,
    const NfpRunControl* control
) {
    std::vector<Polygon> cleaned;
    cleaned.reserve(polygons.size());

    for (const auto& polygon : polygons) {
        Polygon p = cleanPolygon(polygon);
        if (p.size() < 3 || std::abs(signedArea(p)) <= kEps) continue;
        if (signedArea(p) < 0.0) std::reverse(p.begin(), p.end());
        cleaned.push_back(std::move(p));
    }

    if (cleaned.empty()) return {};
    if (cleaned.size() == 1) return cleaned;

    const auto boundary = unionBoundaryPieces(cleaned, control);
    return assembleBoundaryLoops(boundary);
}

std::vector<Polygon> conservativeConvexFallback(const Polygon& polygon) {
    Polygon cleaned = cleanPolygon(polygon);
    if (cleaned.size() < 3) return {};

    Polygon hull = convexHull(std::move(cleaned));
    if (hull.size() >= 3 && std::abs(signedArea(hull)) > kEps) {
        return {std::move(hull)};
    }

    const Bounds b = bounds(polygon);
    if (b.width() <= kPointEps || b.height() <= kPointEps) return {};

    return {Polygon{
        {b.minX, b.minY},
        {b.maxX, b.minY},
        {b.maxX, b.maxY},
        {b.minX, b.maxY}
    }};
}

std::vector<Polygon> conservativeNfpFallback(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation
) {
    const auto fixedFallback = conservativeConvexFallback(fixed);
    const auto movingFallback = conservativeConvexFallback(
        rotate(moving, rotation)
    );
    if (fixedFallback.empty() || movingFallback.empty()) return {};

    const auto fallback = minkowskiConvexSum(
        fixedFallback.front(),
        reflected(movingFallback.front())
    );
    if (fallback.size() < 3) return {};
    return {fallback};
}

std::vector<Polygon> decomposeWithFallback(
    const Polygon& polygon,
    const NfpRunControl* control
) {
    if (control && control->stop()) return conservativeConvexFallback(polygon);
    if (control && polygon.size() > control->maxInputVertices) {
        control->complexityFallback();
        return conservativeConvexFallback(polygon);
    }
    auto pieces = convexDecompose(polygon, control);
    if (!pieces.empty() &&
        (!control || pieces.size() <= control->maxConvexPieces)) return pieces;
    if (control && pieces.size() > control->maxConvexPieces) {
        control->complexityFallback();
    }

    // A malformed/near-degenerate contour must not collapse the NFP stage to
    // zero candidates. The convex-hull fallback is deliberately conservative:
    // it may reject some valid concave placements, but it never authorizes an
    // overlap. The nesting layer still performs exact true-shape validation
    // and can enter its geometric grid recovery path when needed.
    return conservativeConvexFallback(polygon);
}

std::vector<Polygon> computeUnionNfp(
    const Polygon& fixed,
    const Polygon& moving,
    const NfpRunControl* control
) {
    if (control && control->stop()) {
        return conservativeNfpFallback(fixed, moving, 0);
    }
    const auto fixedPieces = decomposeWithFallback(fixed, control);
    const auto movingPieces = decomposeWithFallback(moving, control);

    std::vector<Polygon> pairwise;
    pairwise.reserve(fixedPieces.size() * movingPieces.size());

    for (const auto& fixedPiece : fixedPieces) {
        if (control && control->stop()) {
            return conservativeNfpFallback(fixed, moving, 0);
        }
        if (control && pairwise.size() >= control->maxPairwisePolygons) {
            control->complexityFallback();
            return conservativeNfpFallback(fixed, moving, 0);
        }
        for (const auto& movingPiece : movingPieces) {
            if (control && control->stop()) {
                return conservativeNfpFallback(fixed, moving, 0);
            }
            const auto reflectedPiece = reflected(movingPiece);
            const auto nfp = minkowskiConvexSum(
                fixedPiece,
                reflectedPiece
            );
            if (nfp.size() >= 3) {
                pairwise.push_back(nfp);
                if (control && pairwise.size() >= control->maxPairwisePolygons) {
                    control->complexityFallback();
                    return conservativeNfpFallback(fixed, moving, 0);
                }
            }
        }
    }

    if (control && control->stop()) {
        return conservativeNfpFallback(fixed, moving, 0);
    }
    auto unionResult = unionPolygons(pairwise, control);
    if (!unionResult.empty()) return unionResult;

    // Last-resort conservative NFP. This keeps the placement pipeline alive
    // for numerically pathological contours instead of returning an empty
    // forbidden region and producing parsed=1/candidates=0 diagnostics.
    return conservativeNfpFallback(fixed, moving, 0);
}

} // namespace

std::vector<Polygon> convexDecompose(
    const Polygon& input,
    const NfpRunControl* control
) {
    Polygon polygon = cleanPolygon(input);
    if (polygon.size() < 3) return {};

    // DXF chains often contain runs of collinear segments. Removing only
    // collinear interior vertices makes ear clipping deterministic without
    // changing the represented simple polygon.
    bool removedCollinear = true;
    while (removedCollinear && polygon.size() > 3) {
        if (control && control->stop()) return {};
        removedCollinear = false;
        for (std::size_t i = 0; i < polygon.size(); ++i) {
            if (control && control->stop()) return {};
            const std::size_t prev =
                (i + polygon.size() - 1) % polygon.size();
            const std::size_t next =
                (i + 1) % polygon.size();

            if (std::abs(cross(
                    polygon[prev],
                    polygon[i],
                    polygon[next]
                )) <= 1e-10) {
                polygon.erase(
                    polygon.begin() +
                    static_cast<std::ptrdiff_t>(i)
                );
                removedCollinear = true;
                break;
            }
        }
    }

    if (polygon.size() < 3) return {};

    if (signedArea(polygon) < 0.0) {
        std::reverse(polygon.begin(), polygon.end());
    }

    std::vector<std::size_t> indices(polygon.size());
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        indices[i] = i;
    }

    std::vector<Polygon> triangles;
    triangles.reserve(polygon.size() - 2);

    std::size_t guard = 0;
    while (indices.size() > 3 &&
           guard++ < polygon.size() * polygon.size()) {
        if (control && control->stop()) return {};
        bool clipped = false;

        for (std::size_t i = 0; i < indices.size(); ++i) {
            if (control && control->stop()) return {};
            const std::size_t ia =
                indices[(i + indices.size() - 1) % indices.size()];
            const std::size_t ib = indices[i];
            const std::size_t ic = indices[(i + 1) % indices.size()];

            const Point a = polygon[ia];
            const Point b = polygon[ib];
            const Point c = polygon[ic];

            if (cross(a, b, c) <= kEps) continue;

            bool containsOther = false;
            for (const auto idx : indices) {
                if (control && control->stop()) return {};
                if (idx == ia || idx == ib || idx == ic) continue;
                if (pointInTriangle(polygon[idx], a, b, c)) {
                    containsOther = true;
                    break;
                }
            }

            if (containsOther) continue;

            triangles.push_back({a, b, c});
            indices.erase(
                indices.begin() + static_cast<std::ptrdiff_t>(i)
            );
            clipped = true;
            break;
        }

        if (!clipped) return {};
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
    const Polygon& moving,
    int rotation,
    double clearanceMm,
    const NfpRunControl* control
) {
    const int normalizedRotation =
        ((rotation % 360) + 360) % 360;

    if (control && control->stop()) {
        return conservativeNfpFallback(fixed, moving, normalizedRotation);
    }

    const std::string key = makeCacheKey(
        fixed,
        moving,
        normalizedRotation,
        clearanceMm
    );

    CacheStore& store = cacheStore();
    std::vector<Polygon> cachedPolygons;
    bool cacheHit = false;
    {
        std::lock_guard<std::mutex> lock(store.mutex);
        const auto it = store.entries.find(key);
        if (it != store.entries.end()) {
            ++store.hits;
            if (control && control->cacheHitCount) {
                ++(*control->cacheHitCount);
            }
            if (validCachedNfp(it->second.polygons)) {
                cachedPolygons = it->second.polygons;
                cacheHit = true;
            } else {
                // Never serve malformed data from a cache entry. Erase it and
                // recompute from geometry rather than converting corruption
                // into a silent "no candidates" result.
                store.entries.erase(it);
            }
        } else {
            ++store.misses;
            if (control && control->cacheMissCount) {
                ++(*control->cacheMissCount);
            }
        }
    }

    const auto [fixedCanonical, fixedOrigin] = canonicalize(fixed);
    const auto [movingCanonical, movingOrigin] =
        canonicalize(rotate(moving, normalizedRotation));

    if (cacheHit) {
        std::vector<Polygon> restored;
        restored.reserve(cachedPolygons.size());

        const double dx = fixedOrigin.x - movingOrigin.x;
        const double dy = fixedOrigin.y - movingOrigin.y;

        for (const auto& polygon : cachedPolygons) {
            restored.push_back(translate(polygon, dx, dy));
        }
        return restored;
    }

    const auto computed = computeUnionNfp(
        fixedCanonical,
        movingCanonical,
        control
    );

    // A deadline is request-scoped and must never poison the shared cache with
    // an incomplete/partial NFP. Return a conservative fallback and leave the
    // normal cache untouched when the guard stopped the computation.
    if (control && control->stop()) {
        return conservativeNfpFallback(fixed, moving, normalizedRotation);
    }

    {
        std::lock_guard<std::mutex> lock(store.mutex);
        if (store.entries.size() >= kMaxCacheEntries) {
            store.entries.erase(store.entries.begin());
        }
        store.entries.emplace(
            key,
            CacheData{computed}
        );
    }

    const double dx = fixedOrigin.x - movingOrigin.x;
    const double dy = fixedOrigin.y - movingOrigin.y;

    std::vector<Polygon> restored;
    restored.reserve(computed.size());
    for (const auto& polygon : computed) {
        restored.push_back(translate(polygon, dx, dy));
    }

    return restored;
}

double segmentLength(Point a, Point b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

bool clipSegmentToRect(
    Point& a,
    Point& b,
    double minX,
    double minY,
    double maxX,
    double maxY
) {
    const Point originalA = a;
    const Point originalB = b;
    const double dx = originalB.x - originalA.x;
    const double dy = originalB.y - originalA.y;

    double t0 = 0.0;
    double t1 = 1.0;

    auto clip = [&](double p, double q) {
        if (std::abs(p) <= kEps) return q >= 0.0;

        const double r = q / p;
        if (p < 0.0) {
            if (r > t1) return false;
            if (r > t0) t0 = r;
        } else {
            if (r < t0) return false;
            if (r < t1) t1 = r;
        }
        return true;
    };

    if (!clip(-dx, originalA.x - minX)) return false;
    if (!clip( dx, maxX - originalA.x)) return false;
    if (!clip(-dy, originalA.y - minY)) return false;
    if (!clip( dy, maxY - originalA.y)) return false;

    if (t1 < t0) return false;

    a = {
        originalA.x + dx * t0,
        originalA.y + dy * t0
    };
    b = {
        originalA.x + dx * t1,
        originalA.y + dy * t1
    };

    return segmentLength(a, b) > kPointEps;
}

Point outwardOffsetDirection(
    Point a,
    Point b,
    double area
) {
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length = std::hypot(dx, dy);
    if (length <= kEps) return {};

    if (area >= 0.0) {
        return {dy / length, -dx / length};
    }
    return {-dy / length, dx / length};
}

std::vector<FeasibilitySegment> offsetBoundary(
    const Polygon& polygon,
    double offset,
    double minX,
    double minY,
    double maxX,
    double maxY
) {
    std::vector<FeasibilitySegment> result;
    if (polygon.size() < 2) return result;

    const double area = signedArea(polygon);
    const double safeOffset = std::max(1e-7, offset);

    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Point p0 = polygon[i];
        const Point p1 = polygon[(i + 1) % polygon.size()];
        const Point normal = outwardOffsetDirection(p0, p1, area);

        Point a{
            p0.x + normal.x * safeOffset,
            p0.y + normal.y * safeOffset
        };
        Point b{
            p1.x + normal.x * safeOffset,
            p1.y + normal.y * safeOffset
        };

        if (!clipSegmentToRect(
                a,
                b,
                minX,
                minY,
                maxX,
                maxY
            )) {
            continue;
        }

        if (segmentLength(a, b) > kPointEps) {
            result.push_back({a, b});
        }
    }

    return result;
}

std::vector<Point> noFitVertices(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation,
    double clearanceMm,
    const NfpRunControl* control
) {
    std::vector<Point> vertices;

    const auto polygons = noFitPolygons(
        fixed,
        moving,
        rotation,
        clearanceMm,
        control
    );

    vertices.reserve(
        polygons.size() * 16
    );

    for (const auto& polygon : polygons) {
        vertices.insert(
            vertices.end(),
            polygon.begin(),
            polygon.end()
        );
    }

    if (clearanceMm > 0.0) {
        // Clearance remains enforced exactly by nesting collision tests.
        // These offset probes improve the chance of hitting a valid
        // clearance-feasible contact location without changing NFP topology.
        const std::array<Point, 8> directions{{
            {1.0, 0.0},
            {-1.0, 0.0},
            {0.0, 1.0},
            {0.0, -1.0},
            {0.7071067811865476, 0.7071067811865476},
            {-0.7071067811865476, 0.7071067811865476},
            {0.7071067811865476, -0.7071067811865476},
            {-0.7071067811865476, -0.7071067811865476}
        }};

        const std::size_t baseCount = vertices.size();
        for (std::size_t i = 0; i < baseCount; ++i) {
            for (const auto direction : directions) {
                vertices.push_back({
                    vertices[i].x + direction.x * clearanceMm,
                    vertices[i].y + direction.y * clearanceMm
                });
            }
        }
    }

    std::sort(vertices.begin(), vertices.end(), [](const Point& a, const Point& b) {
        if (std::abs(a.x - b.x) > kPointEps) return a.x < b.x;
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


FeasibilityRegion feasibilityRegion(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation,
    double minX,
    double minY,
    double maxX,
    double maxY,
    double clearanceMm,
    const NfpRunControl* control
) {
    FeasibilityRegion region;

    if (maxX < minX - kPointEps || maxY < minY - kPointEps ||
        fixed.size() < 3 || moving.size() < 3) {
        return region;
    }

    // NFP is the forbidden translation region. Its exterior boundary,
    // offset outward by the required technological gap, is the continuous
    // contact boundary of the feasible region.
    const auto forbidden = noFitPolygons(
        fixed,
        moving,
        rotation,
        0.0,
        control
    );

    const double safeGap = std::max(1e-7, clearanceMm);

    // Edge offsets alone do not describe the complete Euclidean clearance
    // boundary at NFP vertices. Add sampled circular joins around each
    // forbidden vertex. Candidates remain subject to exact collision checks.
    for (const auto& polygon : forbidden) {
        const auto segments = offsetBoundary(
            polygon,
            safeGap,
            minX,
            minY,
            maxX,
            maxY
        );
        region.boundary.insert(
            region.boundary.end(),
            segments.begin(),
            segments.end()
        );

        if (safeGap > 1e-7) {
            constexpr int kArcSamples = 16;
            const double twoPi =
                2.0 * 3.14159265358979323846;

            for (const auto& vertex : polygon) {
                Point previous{};
                bool hasPrevious = false;

                for (int sample = 0;
                     sample <= kArcSamples;
                     ++sample) {
                    const double angle =
                        twoPi *
                        static_cast<double>(sample) /
                        static_cast<double>(kArcSamples);

                    const Point current{
                        vertex.x + std::cos(angle) * safeGap,
                        vertex.y + std::sin(angle) * safeGap
                    };

                    if (hasPrevious) {
                        Point a = previous;
                        Point b = current;
                        if (clipSegmentToRect(
                                a,
                                b,
                                minX,
                                minY,
                                maxX,
                                maxY
                            )) {
                            region.boundary.push_back({a, b});
                        }
                    }

                    previous = current;
                    hasPrevious = true;
                }
            }
        }
    }

    const Polygon sheet{
        {minX, minY},
        {maxX, minY},
        {maxX, maxY},
        {minX, maxY}
    };

    for (std::size_t i = 0; i < sheet.size(); ++i) {
        region.sheetBoundary.push_back({
            sheet[i],
            sheet[(i + 1) % sheet.size()]
        });
    }

    return region;
}

std::vector<Point> pointsOnFeasibilityBoundary(
    const FeasibilityRegion& region,
    double spacingMm,
    std::size_t maxPoints,
    bool includeSheetBoundary
) {
    struct SegmentWork {
        FeasibilitySegment segment;
        double length{};
        double bestY{};
        double bestX{};
    };

    const double spacing = std::max(0.01, spacingMm);
    const std::size_t budget = std::max<std::size_t>(4, maxPoints);

    std::vector<SegmentWork> segments;
    segments.reserve(
        region.boundary.size() +
        (includeSheetBoundary ? region.sheetBoundary.size() : 0)
    );

    auto collect = [&](const std::vector<FeasibilitySegment>& source) {
        for (const auto& segment : source) {
            const double length = segmentLength(segment.a, segment.b);
            if (length <= kPointEps) continue;

            const Point* first = &segment.a;
            const Point* second = &segment.b;

            if (std::tie(second->y, second->x) <
                std::tie(first->y, first->x)) {
                std::swap(first, second);
            }

            segments.push_back({
                segment,
                length,
                first->y,
                first->x
            });
        }
    };

    collect(region.boundary);
    if (includeSheetBoundary) collect(region.sheetBoundary);

    if (segments.empty()) return {};

    std::vector<Point> points;
    points.reserve(std::min<std::size_t>(budget, segments.size() * 3));

    auto appendPoint = [&](Point p) {
        points.push_back(p);
    };

    // When the number of segments exceeds the global budget, do not
    // discard whole spatial regions merely because their segments are short.
    // The candidate budget is deliberately split between:
    //   1) spatially stratified segments covering the full boundary order;
    //   2) the longest segments, which preserve dense coverage on large
    //      feasibility edges.
    // Each selected segment contributes an endpoint and, when useful, an
    // interior midpoint. This keeps the search continuous without allowing
    // one long edge to monopolize the candidate budget.
    if (segments.size() >= budget) {
        const std::size_t selectedSegments =
            std::max<std::size_t>(1, budget / 2);

        std::vector<std::size_t> spatialOrder(segments.size());
        for (std::size_t i = 0; i < segments.size(); ++i) {
            spatialOrder[i] = i;
        }

        std::sort(
            spatialOrder.begin(),
            spatialOrder.end(),
            [&](std::size_t a, std::size_t b) {
                if (std::abs(segments[a].bestY - segments[b].bestY) > kPointEps) {
                    return segments[a].bestY < segments[b].bestY;
                }
                if (std::abs(segments[a].bestX - segments[b].bestX) > kPointEps) {
                    return segments[a].bestX < segments[b].bestX;
                }
                return a < b;
            }
        );

        std::vector<std::size_t> lengthOrder(segments.size());
        for (std::size_t i = 0; i < segments.size(); ++i) {
            lengthOrder[i] = i;
        }

        std::sort(
            lengthOrder.begin(),
            lengthOrder.end(),
            [&](std::size_t a, std::size_t b) {
                if (std::abs(segments[a].length - segments[b].length) > kPointEps) {
                    return segments[a].length > segments[b].length;
                }
                if (std::abs(segments[a].bestY - segments[b].bestY) > kPointEps) {
                    return segments[a].bestY < segments[b].bestY;
                }
                if (std::abs(segments[a].bestX - segments[b].bestX) > kPointEps) {
                    return segments[a].bestX < segments[b].bestX;
                }
                return a < b;
            }
        );

        std::vector<std::size_t> selected;
        selected.reserve(selectedSegments);
        std::vector<bool> selectedFlags(segments.size(), false);

        // First reserve half of the selected slots across the complete
        // spatially ordered boundary. Evenly spaced indices guarantee that
        // short/isolated feasibility segments remain visible.
        const std::size_t spatialSlots =
            std::max<std::size_t>(1, selectedSegments / 2);
        for (std::size_t slot = 0;
             slot < spatialSlots && selected.size() < selectedSegments;
             ++slot) {
            const std::size_t index =
                (slot * segments.size()) /
                std::max<std::size_t>(1, spatialSlots);
            const std::size_t clamped =
                std::min(index, segments.size() - 1);

            if (!selectedFlags[spatialOrder[clamped]]) {
                selected.push_back(spatialOrder[clamped]);
                selectedFlags[spatialOrder[clamped]] = true;
            }
        }

        // Fill the remaining slots with the longest segments, but only after
        // the global boundary coverage has been reserved.
        for (const auto index : lengthOrder) {
            if (selected.size() >= selectedSegments) break;
            if (selectedFlags[index]) continue;
            selected.push_back(index);
            selectedFlags[index] = true;
        }

        std::sort(
            selected.begin(),
            selected.end(),
            [&](std::size_t a, std::size_t b) {
                if (std::abs(segments[a].bestY - segments[b].bestY) > kPointEps) {
                    return segments[a].bestY < segments[b].bestY;
                }
                if (std::abs(segments[a].bestX - segments[b].bestX) > kPointEps) {
                    return segments[a].bestX < segments[b].bestX;
                }
                return a < b;
            }
        );

        for (const auto index : selected) {
            const auto& work = segments[index];
            const Point a = work.segment.a;
            const Point b = work.segment.b;

            if (std::tie(b.y, b.x) < std::tie(a.y, a.x)) {
                appendPoint(b);
            } else {
                appendPoint(a);
            }
        }

        for (const auto index : selected) {
            if (points.size() >= budget) break;
            const auto& work = segments[index];
            if (work.length + kPointEps < spacing) continue;

            appendPoint({
                (work.segment.a.x + work.segment.b.x) * 0.5,
                (work.segment.a.y + work.segment.b.y) * 0.5
            });
        }
    } else {
        for (const auto& work : segments) {
            const Point a = work.segment.a;
            const Point b = work.segment.b;

            if (std::tie(b.y, b.x) < std::tie(a.y, a.x)) {
                appendPoint(b);
            } else {
                appendPoint(a);
            }
        }

        if (points.size() < budget) {
            const std::size_t remaining = budget - points.size();

            double totalLength = 0.0;
            for (const auto& work : segments) {
                totalLength += work.length;
            }

            std::vector<std::size_t> allocations(segments.size(), 0);

            if (totalLength > kPointEps) {
                std::size_t requestedTotal = 0;
                std::vector<std::size_t> requested(segments.size(), 0);

                for (std::size_t i = 0; i < segments.size(); ++i) {
                    // Endpoints are already present. spacingMm controls only
                    // the additional interior coverage requested on each
                    // segment.
                    const auto count =
                        segments[i].length > spacing + kPointEps
                            ? static_cast<std::size_t>(
                                  std::floor(
                                      segments[i].length / spacing
                                  )
                              ) - 1
                            : 0;

                    requested[i] = count;
                    requestedTotal += count;
                }

                if (requestedTotal <= remaining) {
                    allocations = requested;
                } else {
                    std::size_t allocated = 0;
                    for (std::size_t i = 0; i < segments.size(); ++i) {
                        const auto count = static_cast<std::size_t>(
                            std::floor(
                                remaining *
                                (static_cast<double>(requested[i]) /
                                 static_cast<double>(
                                     std::max<std::size_t>(
                                         1,
                                         requestedTotal
                                     )
                                 ))
                            )
                        );
                        allocations[i] = count;
                        allocated += count;
                    }

                    // Long/requested-dense segments receive the remainder
                    // first. This keeps the candidate distribution
                    // deterministic across repeated calculations.
                    std::vector<std::size_t> order(segments.size());
                    for (std::size_t i = 0; i < segments.size(); ++i) {
                        order[i] = i;
                    }

                    std::sort(
                        order.begin(),
                        order.end(),
                        [&](std::size_t a, std::size_t b) {
                            if (requested[a] != requested[b]) {
                                return requested[a] > requested[b];
                            }
                            if (std::abs(
                                    segments[a].length -
                                    segments[b].length
                                ) > kPointEps) {
                                return segments[a].length >
                                       segments[b].length;
                            }
                            return a < b;
                        }
                    );

                    for (const auto index : order) {
                        if (allocated >= remaining) break;
                        if (allocations[index] >= requested[index]) continue;
                        ++allocations[index];
                        ++allocated;
                    }
                }
            }

            for (std::size_t i = 0; i < segments.size(); ++i) {
                const auto count = allocations[i];
                if (count == 0) continue;

                const Point a = segments[i].segment.a;
                const Point b = segments[i].segment.b;

                // Count interior samples only. Endpoints were already added
                // above, so every extra point contributes new coverage.
                for (std::size_t k = 1; k <= count; ++k) {
                    const double fraction =
                        static_cast<double>(k) /
                        static_cast<double>(count + 1);

                    appendPoint({
                        a.x + (b.x - a.x) * fraction,
                        a.y + (b.y - a.y) * fraction
                    });
                }
            }
        }
    }

    // A very coarse requested spacing should still expose the midpoint of a
    // segment. This is important when both endpoints fail collision checks
    // against other already-placed parts while an interior portion is valid.
    if (points.size() < budget) {
        for (const auto& work : segments) {
            if (work.length + kPointEps < spacing) continue;
            if (points.size() >= budget) break;

            points.push_back({
                (work.segment.a.x + work.segment.b.x) * 0.5,
                (work.segment.a.y + work.segment.b.y) * 0.5
            });
        }
    }

    std::sort(
        points.begin(),
        points.end(),
        [](const Point& a, const Point& b) {
            if (std::abs(a.y - b.y) > kPointEps) return a.y < b.y;
            return a.x < b.x;
        }
    );

    points.erase(
        std::unique(
            points.begin(),
            points.end(),
            [](const Point& a, const Point& b) {
                return samePoint(a, b);
            }
        ),
        points.end()
    );

    if (points.size() > budget) {
        points.resize(budget);
    }

    (void)spacing;
    return points;
}


void clearCache() {
    CacheStore& store = cacheStore();
    std::lock_guard<std::mutex> lock(store.mutex);
    store.entries.clear();
    store.hits = 0;
    store.misses = 0;
}

CacheStats cacheStats() {
    CacheStore& store = cacheStore();
    std::lock_guard<std::mutex> lock(store.mutex);
    return {
        store.hits,
        store.misses,
        store.entries.size()
    };
}


SearchResult searchFeasibleBoundary(
    const Polygon& fixed,
    const Polygon& moving,
    int rotation,
    double minX,
    double minY,
    double maxX,
    double maxY,
    double clearanceMm,
    const SearchOptions& options,
    const NfpRunControl* control
) {
    SearchResult result;

    if (maxX < minX - kEps || maxY < minY - kEps ||
        fixed.size() < 3 || moving.size() < 3) {
        return result;
    }

    if (control && control->stop()) {
        result.telemetry.stopped = true;
        return result;
    }

    const std::size_t budget = std::max<std::size_t>(
        1,
        options.maxCandidates
    );

    const auto region = feasibilityRegion(
        fixed,
        moving,
        rotation,
        minX,
        minY,
        maxX,
        maxY,
        clearanceMm,
        control
    );

    if (control && control->stop()) {
        result.telemetry.stopped = true;
        return result;
    }

    result.telemetry.boundarySegments =
        region.boundary.size() + (
            options.includeSheetBoundary
                ? region.sheetBoundary.size()
                : 0
        );

    // The boundary sampler is deliberately continuous: it samples segment
    // interiors as well as endpoints. This avoids the historical failure mode
    // where a valid placement exists in the middle of a long feasibility edge
    // but no NFP vertex is feasible.
    auto boundaryPoints = pointsOnFeasibilityBoundary(
        region,
        std::max(0.01, options.boundarySpacingMm),
        budget,
        options.includeSheetBoundary
    );

    result.telemetry.boundarySamples = boundaryPoints.size();
    result.telemetry.generated += boundaryPoints.size();

    std::vector<Point> candidates;
    candidates.reserve(
        std::min<std::size_t>(
            budget * 2,
            boundaryPoints.size() + 64
        )
    );

    auto appendCandidate = [&](Point point) {
        if (point.x < minX - kPointEps ||
            point.x > maxX + kPointEps ||
            point.y < minY - kPointEps ||
            point.y > maxY + kPointEps) {
            return;
        }

        candidates.push_back(point);
    };

    for (const auto point : boundaryPoints) {
        if (control && control->stop()) {
            result.telemetry.stopped = true;
            return result;
        }
        appendCandidate(point);
    }

    if (options.includeNfpVertices && candidates.size() < budget) {
        const auto nfpPolygons = noFitPolygons(
            fixed,
            moving,
            rotation,
            0.0,
            control
        );

        for (const auto& polygon : nfpPolygons) {
            for (const auto point : polygon) {
                if (control && control->stop()) {
                    result.telemetry.stopped = true;
                    return result;
                }
                ++result.telemetry.nfpVertices;
                appendCandidate(point);
                if (candidates.size() >= budget) break;
            }
            if (candidates.size() >= budget) break;
        }
    }

    // Stable spatial deduplication. Quantization is used only for duplicate
    // suppression; the original double coordinates are retained for exact
    // downstream validation.
    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const Point& a, const Point& b) {
            if (std::abs(a.y - b.y) > kPointEps) return a.y < b.y;
            return a.x < b.x;
        }
    );

    candidates.erase(
        std::unique(
            candidates.begin(),
            candidates.end(),
            [](const Point& a, const Point& b) {
                return samePoint(a, b);
            }
        ),
        candidates.end()
    );

    result.telemetry.deduplicated = candidates.size();

    if (candidates.size() > budget) {
        result.telemetry.budgetExceeded = true;

        // Preserve the lower-left ordering used by nesting, but retain
        // spatially distributed samples so one dense boundary segment cannot
        // monopolize the candidate budget.
        const std::size_t keep = budget;
        std::vector<Point> selected;
        selected.reserve(keep);

        if (keep == 1) {
            selected.push_back(candidates.front());
        } else {
            for (std::size_t i = 0; i < keep; ++i) {
                const std::size_t index =
                    (i * (candidates.size() - 1)) /
                    (keep - 1);
                selected.push_back(candidates[index]);
            }
        }

        candidates.swap(selected);
    }

    result.telemetry.generated += candidates.size();

    for (const auto point : candidates) {
        if (control && control->stop()) {
            result.telemetry.stopped = true;
            break;
        }

        if (options.isFeasible) {
            ++result.telemetry.exactChecks;
            if (!options.isFeasible(point)) {
                ++result.telemetry.rejected;
                continue;
            }
        }

        result.points.push_back(point);
        ++result.telemetry.feasible;

        if (result.points.size() >= budget) break;
    }

    return result;
}


} // namespace sheetnest::nfp
