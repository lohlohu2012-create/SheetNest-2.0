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
constexpr std::size_t kMaxCacheEntries = 2048;

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

    const auto b = bounds(polygon);
    Polygon normalized = translate(
        polygon,
        -b.minX,
        -b.minY
    );
    return {std::move(normalized), {b.minX, b.minY}};
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
    const std::vector<Polygon>& polygons
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
        const Polygon polygon = cleanPolygon(polygons[polyIndex]);
        if (polygon.size() < 3) continue;

        const double area = signedArea(polygon);
        if (std::abs(area) <= kEps) continue;

        for (std::size_t edgeIndex = 0; edgeIndex < polygon.size(); ++edgeIndex) {
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
    const std::vector<Polygon>& polygons
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

    const auto boundary = unionBoundaryPieces(cleaned);
    return assembleBoundaryLoops(boundary);
}

std::vector<Polygon> computeUnionNfp(
    const Polygon& fixed,
    const Polygon& moving
) {
    const auto fixedPieces = convexDecompose(fixed);
    const auto movingPieces = convexDecompose(moving);

    std::vector<Polygon> pairwise;
    pairwise.reserve(fixedPieces.size() * movingPieces.size());

    for (const auto& fixedPiece : fixedPieces) {
        for (const auto& movingPiece : movingPieces) {
            const auto reflectedPiece = reflected(movingPiece);
            const auto nfp = minkowskiConvexSum(
                fixedPiece,
                reflectedPiece
            );
            if (nfp.size() >= 3) pairwise.push_back(nfp);
        }
    }

    return unionPolygons(pairwise);
}

} // namespace

std::vector<Polygon> convexDecompose(const Polygon& input) {
    Polygon polygon = cleanPolygon(input);
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
        bool clipped = false;

        for (std::size_t i = 0; i < indices.size(); ++i) {
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
    double clearanceMm
) {
    const int normalizedRotation =
        ((rotation % 360) + 360) % 360;

    const std::string key = makeCacheKey(
        fixed,
        moving,
        normalizedRotation,
        clearanceMm
    );

    CacheStore& store = cacheStore();
    {
        std::lock_guard<std::mutex> lock(store.mutex);
        const auto it = store.entries.find(key);
        if (it != store.entries.end()) {
            ++store.hits;

            const auto [fixedCanonical, fixedOrigin] = canonicalize(fixed);
            const auto [movingCanonical, movingOrigin] =
                canonicalize(rotate(moving, normalizedRotation));

            (void)fixedCanonical;
            return [&] {
                std::vector<Polygon> restored;
                restored.reserve(it->second.polygons.size());

                const double dx = fixedOrigin.x - movingOrigin.x;
                const double dy = fixedOrigin.y - movingOrigin.y;

                for (const auto& polygon : it->second.polygons) {
                    restored.push_back(translate(polygon, dx, dy));
                }
                return restored;
            }();
        }

        ++store.misses;
    }

    const auto [fixedCanonical, fixedOrigin] = canonicalize(fixed);
    const auto [movingCanonical, movingOrigin] =
        canonicalize(rotate(moving, normalizedRotation));

    const auto computed = computeUnionNfp(
        fixedCanonical,
        movingCanonical
    );

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
    double clearanceMm
) {
    std::vector<Point> vertices;

    const auto polygons = noFitPolygons(
        fixed,
        moving,
        rotation,
        clearanceMm
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
    double clearanceMm
) {
    FeasibilityRegion region;

    if (maxX <= minX || maxY <= minY ||
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
        0.0
    );

    const double safeGap = std::max(1e-7, clearanceMm);

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
    double spacingMm
) {
    std::vector<Point> points;

    const double spacing = std::max(0.01, spacingMm);

    auto addSegment = [&](const FeasibilitySegment& segment) {
        const double length = segmentLength(segment.a, segment.b);
        if (length <= kPointEps) return;

        const std::size_t count = std::max<std::size_t>(
            2,
            static_cast<std::size_t>(std::ceil(length / spacing)) + 1
        );

        for (std::size_t i = 0; i < count; ++i) {
            const double t =
                static_cast<double>(i) /
                static_cast<double>(count - 1);

            points.push_back({
                segment.a.x +
                    (segment.b.x - segment.a.x) * t,
                segment.a.y +
                    (segment.b.y - segment.a.y) * t
            });
        }

        // Exact projection candidates avoid making the continuous boundary
        // dependent on the sampling grid when the objective prefers a low
        // or left-most point on an otherwise long feasible edge.
        const double dx = segment.b.x - segment.a.x;
        const double dy = segment.b.y - segment.a.y;
        const double len2 = dx * dx + dy * dy;

        if (len2 > kEps) {
            const double tX = std::clamp(
                -segment.a.x * dx / len2,
                0.0,
                1.0
            );
            const double tY = std::clamp(
                -segment.a.y * dy / len2,
                0.0,
                1.0
            );

            for (const double t : {tX, tY, 0.5}) {
                points.push_back({
                    segment.a.x + dx * t,
                    segment.a.y + dy * t
                });
            }
        }
    };

    for (const auto& segment : region.boundary) addSegment(segment);
    for (const auto& segment : region.sheetBoundary) addSegment(segment);

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

} // namespace sheetnest::nfp
