#include "sheetnest/nesting.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

namespace sheetnest {
namespace {

constexpr double kEps = 1e-7;

struct PlacedShape {
    Polygon outer;
    std::vector<Polygon> holes;
    Placement placement;
};

struct SheetState {
    std::vector<PlacedShape> shapes;
    std::vector<Placement> placements;
    double placedArea{};
};

struct Candidate {
    double x{};
    double y{};
    double scoreY{};
    double scoreX{};
};

double signedArea(const Polygon& p) {
    if (p.size() < 3) return 0.0;
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto& u = p[i];
        const auto& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return a * 0.5;
}

double materialArea(const Part& part) {
    double area = std::abs(signedArea(part.outer));
    for (const auto& hole : part.holes) {
        area -= std::abs(signedArea(hole));
    }
    return std::max(0.0, area);
}

Polygon transformPolygon(const Polygon& p, int rotation, double x, double y) {
    return translate(rotate(p, rotation), x, y);
}

PlacedShape transformed(const Instance& instance, int rotation, double x, double y) {
    PlacedShape result;
    result.outer = transformPolygon(instance.part.outer, rotation, x, y);
    result.placement = {instance.id, x, y, rotation};

    result.holes.reserve(instance.part.holes.size());
    for (const auto& hole : instance.part.holes) {
        result.holes.push_back(transformPolygon(hole, rotation, x, y));
    }
    return result;
}

bool pointInMaterial(const PlacedShape& shape, const Point& p) {
    if (!pointInPolygon(p, shape.outer)) return false;
    for (const auto& hole : shape.holes) {
        if (pointInPolygon(p, hole)) return false;
    }
    return true;
}

double pointSegmentDistance(Point p, Point a, Point b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    if (len2 <= kEps) return std::hypot(p.x - a.x, p.y - a.y);

    double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2;
    t = std::clamp(t, 0.0, 1.0);

    const double qx = a.x + t * vx;
    const double qy = a.y + t * vy;
    return std::hypot(p.x - qx, p.y - qy);
}

double segmentDistance(Point a, Point b, Point c, Point d) {
    if (polygonsIntersect({a, b}, {c, d})) return 0.0;

    return std::min({
        pointSegmentDistance(a, c, d),
        pointSegmentDistance(b, c, d),
        pointSegmentDistance(c, a, b),
        pointSegmentDistance(d, a, b)
    });
}

double boundaryDistance(const Polygon& a, const Polygon& b) {
    if (a.empty() || b.empty()) return std::numeric_limits<double>::infinity();

    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Point a0 = a[i];
        const Point a1 = a[(i + 1) % a.size()];
        for (std::size_t j = 0; j < b.size(); ++j) {
            const Point b0 = b[j];
            const Point b1 = b[(j + 1) % b.size()];
            best = std::min(best, segmentDistance(a0, a1, b0, b1));
        }
    }
    return best;
}

double minBoundaryDistance(const PlacedShape& a, const PlacedShape& b) {
    double best = boundaryDistance(a.outer, b.outer);
    for (const auto& ah : a.holes) {
        best = std::min(best, boundaryDistance(ah, b.outer));
        for (const auto& bh : b.holes) best = std::min(best, boundaryDistance(ah, bh));
    }
    for (const auto& bh : b.holes) best = std::min(best, boundaryDistance(a.outer, bh));
    return best;
}

bool materialOverlap(const PlacedShape& a, const PlacedShape& b) {
    if (polygonsIntersect(a.outer, b.outer)) return true;

    for (const auto& p : a.outer) {
        if (pointInMaterial(b, p)) return true;
    }
    for (const auto& p : b.outer) {
        if (pointInMaterial(a, p)) return true;
    }

    return false;
}

bool conflict(const PlacedShape& a, const PlacedShape& b, double gap) {
    if (materialOverlap(a, b)) return true;
    return minBoundaryDistance(a, b) + kEps < std::max(0.0, gap);
}

Bounds combinedBounds(const std::vector<PlacedShape>& shapes, const PlacedShape& extra) {
    Bounds result = bounds(extra.outer);
    for (const auto& shape : shapes) {
        const auto b = bounds(shape.outer);
        result.minX = std::min(result.minX, b.minX);
        result.minY = std::min(result.minY, b.minY);
        result.maxX = std::max(result.maxX, b.maxX);
        result.maxY = std::max(result.maxY, b.maxY);
    }
    return result;
}

bool fitsSheet(
    const PlacedShape& shape,
    const Sheet& sheet,
    double marginMm
) {
    const auto b = bounds(shape.outer);
    const double margin = std::max(0.0, marginMm);

    return b.minX >= margin - kEps &&
           b.minY >= margin - kEps &&
           b.maxX <= sheet.width - margin + kEps &&
           b.maxY <= sheet.height - margin + kEps;
}

std::vector<Candidate> candidatesFor(
    const Polygon& part,
    const SheetState& sheet,
    const Sheet& sheetSize,
    double gap,
    double margin
) {
    const auto pb = bounds(part);
    const double minX = margin - pb.minX;
    const double minY = margin - pb.minY;

    std::vector<Candidate> result;
    result.reserve(64);
    result.push_back({minX, minY, margin, margin});

    const double g = std::max(0.0, gap);

    for (const auto& placed : sheet.shapes) {
        const auto b = bounds(placed.outer);

        const double xs[] = {
            b.minX - pb.maxX - g,
            b.maxX - pb.minX + g,
            b.minX - pb.maxX,
            b.maxX - pb.minX
        };
        const double ys[] = {
            b.minY - pb.maxY - g,
            b.maxY - pb.minY + g,
            b.minY - pb.maxY,
            b.maxY - pb.minY
        };

        for (double x : xs) {
            for (double y : ys) {
                result.push_back({x, y, y, x});
            }
        }

        // Vertex-to-vertex candidates expose concave interlocking positions
        // that bounding-box stepping cannot see.
        for (const auto& pv : part) {
            for (const auto& qv : placed.outer) {
                result.push_back({
                    qv.x - pv.x,
                    qv.y - pv.y,
                    qv.y - pv.y,
                    qv.x - pv.x
                });
                result.push_back({
                    qv.x - pv.x + g,
                    qv.y - pv.y + g,
                    qv.y - pv.y + g,
                    qv.x - pv.x + g
                });
            }
        }
    }

    std::sort(result.begin(), result.end(), [](const Candidate& a, const Candidate& b) {
        return std::tie(a.scoreY, a.scoreX) < std::tie(b.scoreY, b.scoreX);
    });

    result.erase(
        std::unique(result.begin(), result.end(), [](const Candidate& a, const Candidate& b) {
            return std::abs(a.x - b.x) <= 1e-6 &&
                   std::abs(a.y - b.y) <= 1e-6;
        }),
        result.end()
    );

    if (result.size() > 512) result.resize(512);
    (void)sheetSize;
    return result;
}

bool betterCandidate(const Candidate& a, const Candidate& b) {
    return std::tie(a.scoreY, a.scoreX) < std::tie(b.scoreY, b.scoreX);
}

bool placeOnSheet(
    const Instance& instance,
    const Sheet& sheet,
    const Options& options,
    SheetState& state,
    std::vector<int> rotations
) {
    Candidate best{};
    bool found = false;
    int bestRotation = 0;
    std::vector<PlacedShape> bestShapes;

    for (int rotation : rotations) {
        const auto rotated = rotate(instance.part.outer, rotation);
        const auto rotatedBounds = bounds(rotated);

        for (const auto& candidate : candidatesFor(
                 rotated,
                 state,
                 sheet,
                 options.gapMm,
                 sheet.edgeMarginMm)) {
            const auto shape =
                transformed(instance, rotation, candidate.x, candidate.y);

            if (!fitsSheet(shape, sheet, sheet.edgeMarginMm)) continue;

            bool collision = false;
            for (const auto& existing : state.shapes) {
                if (conflict(shape, existing, options.gapMm)) {
                    collision = true;
                    break;
                }
            }
            if (collision) continue;

            const auto merged = combinedBounds(state.shapes, shape);
            Candidate score = candidate;
            score.scoreY = merged.maxY;
            score.scoreX = merged.maxX;

            if (!found || betterCandidate(score, best)) {
                found = true;
                best = score;
                bestRotation = rotation;
                bestShapes.clear();
            }
        }

        (void)rotatedBounds;
    }

    if (!found) return false;

    const auto chosen =
        transformed(instance, bestRotation, best.x, best.y);
    state.shapes.push_back(chosen);
    state.placements.push_back(chosen.placement);
    state.placedArea += materialArea(instance.part);
    return true;
}

bool betterResult(
    const Result& candidate,
    const Result& best
) {
    if (candidate.unplaced.size() != best.unplaced.size()) {
        return candidate.unplaced.size() < best.unplaced.size();
    }
    if (candidate.sheets.size() != best.sheets.size()) {
        return candidate.sheets.size() < best.sheets.size();
    }
    return candidate.utilization > best.utilization + kEps;
}

Result runAttempt(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    std::vector<std::size_t> order,
    std::mt19937& rng
) {
    Result result;
    std::vector<SheetState> states;
    states.reserve(16);

    if (order.empty()) return result;

    for (std::size_t position = 0; position < order.size(); ++position) {
        const auto& instance = instances[order[position]];

        bool placed = false;
        std::vector<std::pair<std::size_t, int>> sheetOrder;

        for (std::size_t s = 0; s < states.size(); ++s) {
            sheetOrder.push_back({s, 0});
        }

        std::vector<int> rotations = options.rotations;
        if (rotations.empty()) rotations.push_back(0);
        if (position > 0) std::shuffle(rotations.begin(), rotations.end(), rng);

        // Try existing sheets before opening a new one. This encodes the
        // primary objective: minimize the sheet count.
        for (std::size_t s = 0; s < states.size() && !placed; ++s) {
            auto trialRotations = rotations;
            if (s > 0) std::rotate(
                trialRotations.begin(),
                trialRotations.begin() + static_cast<std::ptrdiff_t>(s % trialRotations.size()),
                trialRotations.end()
            );

            placed = placeOnSheet(
                instance,
                sheet,
                options,
                states[s],
                trialRotations
            );
        }

        if (!placed) {
            SheetState state;
            placed = placeOnSheet(
                instance,
                sheet,
                options,
                state,
                rotations
            );
            if (placed) states.push_back(std::move(state));
        }

        if (!placed) result.unplaced.push_back(instance.id);
    }

    result.sheets.reserve(states.size());
    double placedArea = 0.0;
    for (auto& state : states) {
        result.sheets.push_back(std::move(state.placements));
        placedArea += state.placedArea;
    }

    const double sheetArea = std::max(0.0, sheet.width * sheet.height);
    result.utilization =
        (sheetArea > 0.0 && !result.sheets.empty())
            ? placedArea / (sheetArea * result.sheets.size())
            : 0.0;

    return result;
}

} // namespace

Result nest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options
) {
    Result best;
    best.unplaced = {};
    best.utilization = -1.0;

    if (sheet.width <= 0.0 || sheet.height <= 0.0) {
        best.unplaced.reserve(instances.size());
        for (const auto& instance : instances) best.unplaced.push_back(instance.id);
        best.utilization = 0.0;
        return best;
    }

    std::vector<std::size_t> order(instances.size());
    std::iota(order.begin(), order.end(), 0);

    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const double areaA = materialArea(instances[a].part);
        const double areaB = materialArea(instances[b].part);
        if (std::abs(areaA - areaB) > kEps) return areaA > areaB;

        const auto ba = bounds(instances[a].part.outer);
        const auto bb = bounds(instances[b].part.outer);
        return std::max(ba.width(), ba.height()) >
               std::max(bb.width(), bb.height());
    });

    const std::size_t iterations = std::max<std::size_t>(1, std::min(options.iterations, 128));
    std::mt19937 rng(options.seed);

    for (std::size_t attempt = 0; attempt < iterations; ++attempt) {
        auto attemptOrder = order;

        if (attempt > 0) {
            std::shuffle(attemptOrder.begin(), attemptOrder.end(), rng);
        }

        auto candidate = runAttempt(
            instances,
            sheet,
            options,
            std::move(attemptOrder),
            rng
        );

        if (best.utilization < 0.0 || betterResult(candidate, best)) {
            best = std::move(candidate);
        }

        // A feasible single-sheet result with every requested instance is a
        // hard lower bound on the primary objective, so further restarts can
        // only improve secondary utilization.
        if (best.unplaced.empty() && best.sheets.size() == 1) {
            // Keep searching when explicitly requested; the utilization
            // comparison still decides whether another restart is better.
        }
    }

    return best;
}

} // namespace sheetnest
