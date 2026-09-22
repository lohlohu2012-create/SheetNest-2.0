#include "sheetnest/nesting.hpp"
#include "sheetnest/nfp.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numeric>
#include <random>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace sheetnest {
namespace {

constexpr double kEps = 1e-7;

bool shouldStop(const Options& options) {
    return options.control && options.control->shouldStop();
}

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

double orientation(Point a, Point b, Point c) {
    const double v = (b.x - a.x) * (c.y - a.y) -
                     (b.y - a.y) * (c.x - a.x);
    return v;
}

bool onSegment(Point p, Point a, Point b) {
    return std::abs(orientation(a, b, p)) <= 1e-9 &&
           p.x >= std::min(a.x, b.x) - 1e-9 &&
           p.x <= std::max(a.x, b.x) + 1e-9 &&
           p.y >= std::min(a.y, b.y) - 1e-9 &&
           p.y <= std::max(a.y, b.y) + 1e-9;
}

bool segmentsIntersect(Point a, Point b, Point c, Point d) {
    const double o1 = orientation(a, b, c);
    const double o2 = orientation(a, b, d);
    const double o3 = orientation(c, d, a);
    const double o4 = orientation(c, d, b);

    if (((o1 > 0.0 && o2 < 0.0) || (o1 < 0.0 && o2 > 0.0)) &&
        ((o3 > 0.0 && o4 < 0.0) || (o3 < 0.0 && o4 > 0.0))) {
        return true;
    }

    return (std::abs(o1) <= 1e-9 && onSegment(c, a, b)) ||
           (std::abs(o2) <= 1e-9 && onSegment(d, a, b)) ||
           (std::abs(o3) <= 1e-9 && onSegment(a, c, d)) ||
           (std::abs(o4) <= 1e-9 && onSegment(b, c, d));
}

double segmentDistance(Point a, Point b, Point c, Point d) {
    if (segmentsIntersect(a, b, c, d)) return 0.0;

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
    // Outer/outer containment by itself is not material overlap: the smaller
    // shape may legitimately lie inside a hole of the other part.
    if (polygonsIntersect(a.outer, b.outer)) {
        bool boundaryCrossesMaterial = false;

        // An outer boundary crossing the other outer boundary is always a
        // material conflict. Outer/hole crossings are also conflicts because
        // material enters the void while the other material occupies that area.
        if (boundaryDistance(a.outer, b.outer) <= kEps) {
            boundaryCrossesMaterial = true;
        }
        for (const auto& hole : b.holes) {
            if (boundaryDistance(a.outer, hole) <= kEps) {
                boundaryCrossesMaterial = true;
                break;
            }
        }
        for (const auto& hole : a.holes) {
            if (boundaryDistance(hole, b.outer) <= kEps) {
                boundaryCrossesMaterial = true;
                break;
            }
        }

        if (boundaryCrossesMaterial) return true;
    }

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
    const std::vector<Polygon>& holes,
    int rotation,
    const SheetState& sheet,
    const Sheet& sheetSize,
    double gap,
    double margin,
    NestingStats* stats,
    const std::shared_ptr<NestingRunControl>& control
) {
    const Polygon rotatedPart = rotate(part, rotation);
    const auto pb = bounds(rotatedPart);
    const double minX = margin - pb.minX;
    const double minY = margin - pb.minY;

    std::vector<Candidate> result;
    result.reserve(64);
    result.push_back({minX, minY, margin, margin});

    const double g = std::max(0.0, gap);

    auto addRingCandidates = [&](const Polygon& ring) {
        const auto b = bounds(ring);

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

        // Vertex-to-vertex candidates remain as a fallback for tight
        // concave interlocking, but cap the quadratic pair explosion on
        // high-resolution/large DXF contours. The NFP boundary is now the
        // primary true-shape candidate source.
        constexpr std::size_t kMaxVertexPairs = 384;
        const std::size_t pairCount =
            std::min(
                kMaxVertexPairs,
                rotatedPart.size() * ring.size()
            );

        if (pairCount == 0) return;

        const std::size_t partStride = std::max<std::size_t>(
            1,
            rotatedPart.size() /
                std::max<std::size_t>(
                    1,
                    static_cast<std::size_t>(
                        std::sqrt(
                            static_cast<double>(pairCount)
                        )
                    )
                )
        );
        const std::size_t ringStride = std::max<std::size_t>(
            1,
            ring.size() /
                std::max<std::size_t>(
                    1,
                    static_cast<std::size_t>(
                        std::sqrt(
                            static_cast<double>(pairCount)
                        )
                    )
                )
        );

        std::size_t emitted = 0;
        for (std::size_t pi = 0;
             pi < rotatedPart.size() && emitted < kMaxVertexPairs;
             pi += partStride) {
            const auto& pv = rotatedPart[pi];

            for (std::size_t qi = 0;
                 qi < ring.size() && emitted < kMaxVertexPairs;
                 qi += ringStride) {
                const auto& qv = ring[qi];

                result.push_back({
                    qv.x - pv.x,
                    qv.y - pv.y,
                    qv.y - pv.y,
                    qv.x - pv.x
                });
                ++emitted;

                if (emitted >= kMaxVertexPairs) break;

                result.push_back({
                    qv.x - pv.x + g,
                    qv.y - pv.y + g,
                    qv.y - pv.y + g,
                    qv.x - pv.x + g
                });
                ++emitted;
            }
        }
    };

    auto addMovingHoleCandidates = [&](const Polygon& movingHole,
                                   const Polygon& fixedRing) {
        if (movingHole.size() < 3 || fixedRing.size() < 3) return;

        constexpr std::size_t kMaxPairs = 256;
        const std::size_t pairCount =
            std::min(kMaxPairs, movingHole.size() * fixedRing.size());
        if (pairCount == 0) return;

        const std::size_t stride =
            std::max<std::size_t>(
                1,
                static_cast<std::size_t>(
                    std::sqrt(
                        static_cast<double>(
                            std::max<std::size_t>(1, pairCount)
                        )
                    )
                )
            );

        std::size_t emitted = 0;
        for (std::size_t hi = 0;
             hi < movingHole.size() && emitted < kMaxPairs;
             hi += stride) {
            const auto& holePoint = movingHole[hi];

            for (std::size_t fi = 0;
                 fi < fixedRing.size() && emitted < kMaxPairs;
                 fi += stride) {
                const auto& fixedPoint = fixedRing[fi];

                result.push_back({
                    fixedPoint.x - holePoint.x,
                    fixedPoint.y - holePoint.y,
                    fixedPoint.y - holePoint.y,
                    fixedPoint.x - holePoint.x
                });
                ++emitted;

                if (g <= 0.0 || emitted >= kMaxPairs) continue;

                result.push_back({
                    fixedPoint.x - holePoint.x + g,
                    fixedPoint.y - holePoint.y + g,
                    fixedPoint.y - holePoint.y + g,
                    fixedPoint.x - holePoint.x + g
                });
                ++emitted;
            }
        }
    };

    for (const auto& placed : sheet.shapes) {
        if (control && control->shouldStop()) break;
        addRingCandidates(placed.outer);

        for (const auto& hole : holes) {
            addMovingHoleCandidates(
                rotate(hole, rotation),
                placed.outer
            );
        }

        // Continuous NFP feasibility boundary: candidate positions are
        // generated along the entire admissible boundary, not only at NFP
        // vertices. Final collision/clearance checks remain authoritative.
        const double placementMinX = margin - pb.minX;
        const double placementMinY = margin - pb.minY;
        const double placementMaxX = sheetSize.width - margin - pb.maxX;
        const double placementMaxY = sheetSize.height - margin - pb.maxY;

        if (placementMaxX >= placementMinX &&
            placementMaxY >= placementMinY) {
            if (control && control->shouldStop()) break;
            if (stats) ++stats->nfpChecks;

            const auto region = nfp::feasibilityRegion(
                placed.outer,
                part,
                rotation,
                placementMinX,
                placementMinY,
                placementMaxX,
                placementMaxY,
                g
            );

            const double characteristicSize = std::sqrt(
                std::max(1.0, std::abs(polygonArea(part)))
            );

            // Large details do not benefit from dense uniform sampling of
            // every long NFP edge. Keep a bounded candidate budget and make
            // the spacing coarser as part size grows. Endpoints and analytic
            // objective-best points remain available even when the budget is
            // tight, while interior samples preserve concave/interlocking
            // opportunities.
            const double boundarySpacing = std::clamp(
                std::max(4.0, characteristicSize * 0.08),
                4.0,
                20.0
            );

            const std::size_t boundaryBudget =
                part.size() > 256
                    ? 72
                    : (part.size() > 128 ? 96 : 128);

            for (const auto& point :
                 nfp::pointsOnFeasibilityBoundary(
                     region,
                     boundarySpacing,
                     boundaryBudget,
                     true
                 )) {
                result.push_back({
                    point.x,
                    point.y,
                    point.y,
                    point.x
                });
            }
        }

        for (const auto& hole : placed.holes) {
            // Holes are usable voids. Generate candidates against their
            // boundaries so smaller parts can interlock inside them.
            addRingCandidates(hole);
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

std::vector<Candidate> fallbackGridCandidates(
    const Polygon& part,
    int rotation,
    const Sheet& sheet,
    double margin
) {
    std::vector<Candidate> result;

    const Polygon rotated = rotate(part, rotation);
    const Bounds pb = bounds(rotated);

    const double minX = margin - pb.minX;
    const double minY = margin - pb.minY;
    const double maxX = sheet.width - margin - pb.maxX;
    const double maxY = sheet.height - margin - pb.maxY;

    if (maxX < minX || maxY < minY) return result;

    // Emergency-only recovery path. NFP candidates are always preferred;
    // every grid point is still validated by the exact polygon predicate.
    constexpr std::size_t kColumns = 64;
    constexpr std::size_t kRows = 32;

    result.reserve(kColumns * kRows);

    for (std::size_t iy = 0; iy < kRows; ++iy) {
        const double ty =
            static_cast<double>(iy) /
            static_cast<double>(kRows - 1);

        for (std::size_t ix = 0; ix < kColumns; ++ix) {
            const double tx =
                static_cast<double>(ix) /
                static_cast<double>(kColumns - 1);

            const double x = minX + (maxX - minX) * tx;
            const double y = minY + (maxY - minY) * ty;
            result.push_back({x, y, y, x});
        }
    }

    std::sort(
        result.begin(),
        result.end(),
        [](const Candidate& a, const Candidate& b) {
            return std::tie(a.scoreY, a.scoreX) <
                   std::tie(b.scoreY, b.scoreX);
        }
    );

    return result;
}

bool placeOnSheet(
    const Instance& instance,
    const Sheet& sheet,
    const Options& options,
    SheetState& state,
    std::vector<int> rotations,
    NestingStats* stats
) {
    Candidate best{};
    bool found = false;
    int bestRotation = 0;
    std::vector<PlacedShape> bestShapes;

    const double margin = std::max(0.0, sheet.edgeMarginMm);

    for (int rotation : rotations) {
        if (shouldStop(options)) return false;
        const auto rotated = rotate(instance.part.outer, rotation);
        const auto rotatedBounds = bounds(rotated);

        // Reject impossible rotations before building NFPs or candidate
        // segments. This is especially valuable when many rotations are
        // requested or the part is close to the sheet size.
        if (rotatedBounds.width() + 2.0 * margin >
                sheet.width + kEps ||
            rotatedBounds.height() + 2.0 * margin >
                sheet.height + kEps) {
            continue;
        }

        for (const auto& candidate : candidatesFor(
                 instance.part.outer,
                 instance.part.holes,
                 rotation,
                 state,
                 sheet,
                 options.gapMm,
                 sheet.edgeMarginMm,
                 stats,
                 options.control)) {
            if (shouldStop(options)) return false;
            if (stats) ++stats->candidateChecks;

            const auto shape =
                transformed(instance, rotation, candidate.x, candidate.y);

            if (!fitsSheet(shape, sheet, sheet.edgeMarginMm)) continue;

            bool collision = false;
            for (const auto& existing : state.shapes) {
                if (stats) ++stats->collisionChecks;
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
    }

    // Recovery for an incomplete NFP boundary sample. This path is only
    // entered after all true-shape candidates failed and is still protected
    // by the exact collision and clearance predicate.
    if (!found) {
        for (int rotation : rotations) {
            for (const auto& candidate : fallbackGridCandidates(
                     instance.part.outer,
                     rotation,
                     sheet,
                     sheet.edgeMarginMm)) {
                if (stats) ++stats->candidateChecks;

                const auto shape =
                    transformed(instance, rotation, candidate.x, candidate.y);

                if (!fitsSheet(shape, sheet, sheet.edgeMarginMm)) continue;

                bool collision = false;
                for (const auto& existing : state.shapes) {
                    if (stats) ++stats->collisionChecks;
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
                }
            }

            if (found) break;
        }
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

const Instance* findInstance(
    const std::vector<Instance>& instances,
    const std::string& id
) {
    for (const auto& instance : instances) {
        if (instance.id == id) return &instance;
    }
    return nullptr;
}

SheetState stateFromPlacements(
    const std::vector<Placement>& placements,
    const std::vector<Instance>& instances
) {
    SheetState state;
    state.shapes.reserve(placements.size());
    state.placements = placements;

    for (const auto& placement : placements) {
        const auto* instance =
            findInstance(instances, placement.id);
        if (!instance) continue;

        state.shapes.push_back(
            transformed(
                *instance,
                placement.rotation,
                placement.x,
                placement.y
            )
        );
        state.placedArea += materialArea(instance->part);
    }

    return state;
}


std::size_t placementIndexById(
    const SheetState& state,
    const std::string& id
) {
    for (std::size_t i = 0; i < state.placements.size(); ++i) {
        if (state.placements[i].id == id) return i;
    }
    return state.placements.size();
}

void erasePlacement(
    SheetState& state,
    std::size_t index,
    const Instance& instance
) {
    if (index >= state.placements.size() ||
        index >= state.shapes.size()) {
        return;
    }

    state.placements.erase(
        state.placements.begin() +
        static_cast<std::ptrdiff_t>(index)
    );
    state.shapes.erase(
        state.shapes.begin() +
        static_cast<std::ptrdiff_t>(index)
    );
    state.placedArea = std::max(
        0.0,
        state.placedArea - materialArea(instance.part)
    );
}

std::vector<std::string> orderedPlacementIds(
    const SheetState& state,
    const std::vector<Instance>& instances
) {
    std::vector<std::string> ids;
    ids.reserve(state.placements.size());

    for (const auto& placement : state.placements) {
        ids.push_back(placement.id);
    }

    std::sort(
        ids.begin(),
        ids.end(),
        [&](const std::string& a, const std::string& b) {
            const auto* ia = findInstance(instances, a);
            const auto* ib = findInstance(instances, b);

            const double aa =
                ia ? materialArea(ia->part) : 0.0;
            const double ab =
                ib ? materialArea(ib->part) : 0.0;

            if (std::abs(aa - ab) > kEps) {
                return aa > ab;
            }
            return a < b;
        }
    );

    return ids;
}

std::vector<int> effectiveRotations(const Options& options) {
    std::vector<int> rotations = options.rotations;
    if (rotations.empty()) {
        rotations.push_back(0);
    }
    return rotations;
}

bool tryPlaceOnExistingSheets(
    const Instance& instance,
    std::vector<SheetState>& states,
    std::size_t sheetLimit,
    const Sheet& sheet,
    const Options& options,
    NestingStats* stats
) {
    std::vector<int> rotations = options.rotations;
    if (rotations.empty()) rotations.push_back(0);

    sheetLimit = std::min(sheetLimit, states.size());

    for (std::size_t targetIndex = 0;
         targetIndex < sheetLimit;
         ++targetIndex) {

        SheetState trial = states[targetIndex];

        if (!placeOnSheet(
                instance,
                sheet,
                options,
                trial,
                rotations,
                stats
            )) {
            continue;
        }

        states[targetIndex] = std::move(trial);
        return true;
    }

    return false;
}

std::vector<std::string> exchangeCandidates(
    const SheetState& target,
    const std::vector<Instance>& instances
) {
    std::vector<std::string> ids;
    ids.reserve(target.placements.size());

    const auto ordered =
        orderedPlacementIds(target, instances);

    // Keep both ends of the area distribution. A blocker may be a tiny
    // detail consuming a narrow gap, but it may also be a large detail whose
    // removal creates a meaningful contiguous placement region.
    constexpr std::size_t kMaxCandidates = 6;

    for (std::size_t i = 0;
         i < ordered.size() &&
         ids.size() < kMaxCandidates / 2;
         ++i) {
        ids.push_back(ordered[i]);
    }

    for (std::size_t i = 0;
         i < ordered.size() &&
         ids.size() < kMaxCandidates;
         ++i) {
        const std::size_t reverseIndex =
            ordered.size() - 1 - i;

        if (std::find(
                ids.begin(),
                ids.end(),
                ordered[reverseIndex]
            ) == ids.end()) {
            ids.push_back(ordered[reverseIndex]);
        }
    }

    return ids;
}

bool relocateEvicted(
    const std::vector<std::string>& evictedIds,
    std::vector<SheetState>& trial,
    std::size_t sourceIndex,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    NestingStats* stats
) {
    for (const auto& id : evictedIds) {
        const auto* instance = findInstance(instances, id);
        if (!instance) return false;

        // Never put an evicted item back into the source sheet: the purpose
        // of this transaction is to make that sheet removable.
        if (!tryPlaceOnExistingSheets(
                *instance,
                trial,
                sourceIndex,
                sheet,
                options,
                stats
            )) {
            return false;
        }
    }

    return true;
}

bool tryExchangeEliminateSheet(
    std::size_t sourceIndex,
    std::vector<SheetState>& states,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    NestingStats* stats
) {
    if (sourceIndex == 0 || sourceIndex >= states.size()) {
        return false;
    }

    const auto sourceIds =
        orderedPlacementIds(states[sourceIndex], instances);

    if (sourceIds.empty()) {
        states.erase(
            states.begin() +
            static_cast<std::ptrdiff_t>(sourceIndex)
        );
        return true;
    }

    constexpr std::size_t kMaxSourceParts = 96;
    constexpr std::size_t kMaxExchangeAttempts = 2500;

    if (sourceIds.size() > kMaxSourceParts) {
        return false;
    }

    std::size_t attempts = 0;
    std::vector<SheetState> working = states;

    auto moveOnePart = [&](const std::string& id) -> bool {
        const auto* instance = findInstance(instances, id);
        if (!instance) return false;

        const auto sourcePos =
            placementIndexById(
                working[sourceIndex],
                id
            );

        if (sourcePos >= working[sourceIndex].placements.size()) {
            return false;
        }

        SheetState directTrial = working[sourceIndex];
        erasePlacement(
            directTrial,
            sourcePos,
            *instance
        );

        // Direct relocation is always cheaper and should be attempted first.
        for (std::size_t targetIndex = 0;
             targetIndex < sourceIndex;
             ++targetIndex) {

            if (++attempts > kMaxExchangeAttempts) {
                return false;
            }

            SheetState targetTrial =
                working[targetIndex];

            if (!placeOnSheet(
                    *instance,
                    sheet,
                    options,
                    targetTrial,
                    effectiveRotations(options),
                    stats
                )) {
                continue;
            }

            auto committed = working;
            committed[targetIndex] =
                std::move(targetTrial);
            committed[sourceIndex] =
                std::move(directTrial);
            working = std::move(committed);
            return true;
        }

        // Now try an exchange: remove one or two blockers from a target,
        // place the source part there, then refill all evicted blockers into
        // other already existing sheets.
        for (std::size_t targetIndex = 0;
             targetIndex < sourceIndex;
             ++targetIndex) {

            const auto candidates =
                exchangeCandidates(
                    working[targetIndex],
                    instances
                );

            for (std::size_t i = 0;
                 i < candidates.size();
                 ++i) {

                // One-blocker exchange.
                for (std::size_t subsetSize = 1;
                     subsetSize <= 2;
                     ++subsetSize) {

                    if (subsetSize == 2 &&
                        i + 1 >= candidates.size()) {
                        break;
                    }

                    if (stats) ++stats->exchangeAttempts;
                    if (++attempts > kMaxExchangeAttempts) {
                        return false;
                    }

                    std::vector<std::string> evicted{
                        candidates[i]
                    };

                    if (subsetSize == 2) {
                        if (candidates[i + 1] == candidates[i]) {
                            continue;
                        }
                        evicted.push_back(
                            candidates[i + 1]
                        );
                    }

                    auto trial = working;

                    bool validEviction = true;
                    for (const auto& evictedId : evicted) {
                        const auto* evictedInstance =
                            findInstance(
                                instances,
                                evictedId
                            );
                        if (!evictedInstance) {
                            validEviction = false;
                            break;
                        }

                        const auto pos =
                            placementIndexById(
                                trial[targetIndex],
                                evictedId
                            );
                        if (pos >= trial[targetIndex].placements.size()) {
                            validEviction = false;
                            break;
                        }

                        erasePlacement(
                            trial[targetIndex],
                            pos,
                            *evictedInstance
                        );
                    }

                    if (!validEviction) continue;

                    auto sourceTrial =
                        trial[sourceIndex];

                    const auto sourcePos =
                        placementIndexById(
                            sourceTrial,
                            id
                        );
                    if (sourcePos >= sourceTrial.placements.size()) {
                        continue;
                    }

                    erasePlacement(
                        sourceTrial,
                        sourcePos,
                        *instance
                    );

                    auto targetTrial =
                        trial[targetIndex];

                    if (!placeOnSheet(
                            *instance,
                            sheet,
                            options,
                            targetTrial,
                            effectiveRotations(options),
                            stats
                        )) {
                        continue;
                    }

                    trial[targetIndex] =
                        std::move(targetTrial);
                    trial[sourceIndex] =
                        std::move(sourceTrial);

                    if (!relocateEvicted(
                            evicted,
                            trial,
                            sourceIndex,
                            instances,
                            sheet,
                            options,
                            stats
                        )) {
                        continue;
                    }

                    working = std::move(trial);
                    if (stats) ++stats->refillMoves;
                    return true;
                }
            }
        }

        return false;
    };

    for (const auto& id : sourceIds) {
        if (shouldStop(options)) return false;
        if (!moveOnePart(id)) {
            return false;
        }
    }

    if (!working[sourceIndex].placements.empty()) {
        return false;
    }

    states = std::move(working);
    states.erase(
        states.begin() +
        static_cast<std::ptrdiff_t>(sourceIndex)
    );
    return true;
}

bool refillExistingSheets(
    std::vector<SheetState>& states,
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    NestingStats* stats
) {
    bool changed = false;
    constexpr int kPasses = 3;

    for (int pass = 0; pass < kPasses; ++pass) {
        if (shouldStop(options)) break;
        bool passChanged = false;

        for (std::size_t sourceIndex = states.size();
             sourceIndex-- > 1;) {
            if (shouldStop(options)) break;

            const auto sourceIds =
                orderedPlacementIds(
                    states[sourceIndex],
                    instances
                );

            for (const auto& id : sourceIds) {
                const auto* instance =
                    findInstance(instances, id);
                if (!instance) continue;

                const auto sourcePos =
                    placementIndexById(
                        states[sourceIndex],
                        id
                    );
                if (sourcePos >= states[sourceIndex].placements.size()) {
                    continue;
                }

                for (std::size_t targetIndex = 0;
                     targetIndex < sourceIndex;
                     ++targetIndex) {

                    SheetState targetTrial =
                        states[targetIndex];

                    if (!placeOnSheet(
                            *instance,
                            sheet,
                            options,
                            targetTrial,
                            effectiveRotations(options),
                            stats
                        )) {
                        continue;
                    }

                    auto committed = states;
                    committed[targetIndex] =
                        std::move(targetTrial);
                    erasePlacement(
                        committed[sourceIndex],
                        sourcePos,
                        *instance
                    );
                    states = std::move(committed);

                    changed = true;
                    passChanged = true;
                    break;
                }
            }
        }

        if (!passChanged) break;
    }

    for (std::size_t i = states.size();
         i-- > 0;) {
        if (states[i].placements.empty()) {
            states.erase(
                states.begin() +
                static_cast<std::ptrdiff_t>(i)
            );
            if (stats) ++stats->sheetsEliminated;
            changed = true;
        }
    }

    return changed;
}

bool compactResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    Result& result,
    NestingStats* stats
) {
    if (result.sheets.size() <= 1) return false;

    std::vector<SheetState> states;
    states.reserve(result.sheets.size());

    for (const auto& placements : result.sheets) {
        states.push_back(
            stateFromPlacements(placements, instances)
        );
    }

    bool changed = false;

    // Work backwards. If every part from the last sheet can be reinserted
    // into earlier sheets, that sheet is redundant and is removed entirely.
    // This directly targets the previous failure mode where a greedy pass
    // opened a new sheet even though an earlier sheet still had usable space.
    for (std::size_t sourceIndex = states.size();
         sourceIndex-- > 1;) {
        if (shouldStop(options)) break;

        SheetState source = states[sourceIndex];
        if (source.shapes.empty()) {
            states.erase(
                states.begin() +
                static_cast<std::ptrdiff_t>(sourceIndex)
            );
            changed = true;
            continue;
        }

        std::vector<std::size_t> order(source.shapes.size());
        std::iota(order.begin(), order.end(), 0);

        std::sort(
            order.begin(),
            order.end(),
            [&](std::size_t a, std::size_t b) {
                const auto* ia =
                    findInstance(instances, source.placements[a].id);
                const auto* ib =
                    findInstance(instances, source.placements[b].id);

                const double aa =
                    ia ? materialArea(ia->part) : 0.0;
                const double ab =
                    ib ? materialArea(ib->part) : 0.0;

                return aa > ab;
            }
        );

        struct Snapshot {
            std::size_t sheetIndex{};
            std::size_t shapeCount{};
            std::size_t placementCount{};
            double placedArea{};
        };

        std::vector<Snapshot> snapshots;
        snapshots.reserve(order.size());

        bool allMoved = true;

        for (const auto sourceShapeIndex : order) {
            const auto& sourcePlacement =
                source.placements[sourceShapeIndex];

            const auto* instance =
                findInstance(instances, sourcePlacement.id);

            if (!instance) {
                allMoved = false;
                break;
            }

            std::size_t bestSheet = states.size();
            PlacedShape bestShape;
            Placement bestPlacement{};
            double bestEnvelopeArea =
                std::numeric_limits<double>::infinity();

            for (std::size_t targetIndex = 0;
                 targetIndex < sourceIndex;
                 ++targetIndex) {

                auto& target = states[targetIndex];

                const std::size_t oldShapeCount =
                    target.shapes.size();
                const std::size_t oldPlacementCount =
                    target.placements.size();
                const double oldPlacedArea =
                    target.placedArea;

                std::vector<int> rotations = options.rotations;
                if (rotations.empty()) rotations.push_back(0);

                if (!placeOnSheet(
                        *instance,
                        sheet,
                        options,
                        target,
                        rotations,
                        stats
                    )) {
                    continue;
                }

                const auto candidateShape =
                    target.shapes.back();

                Bounds envelope =
                    bounds(candidateShape.outer);

                for (std::size_t i = 0;
                     i + 1 < target.shapes.size();
                     ++i) {

                    const auto b =
                        bounds(target.shapes[i].outer);

                    envelope.minX =
                        std::min(envelope.minX, b.minX);
                    envelope.minY =
                        std::min(envelope.minY, b.minY);
                    envelope.maxX =
                        std::max(envelope.maxX, b.maxX);
                    envelope.maxY =
                        std::max(envelope.maxY, b.maxY);
                }

                const double area =
                    envelope.width() * envelope.height();

                if (bestSheet == states.size() ||
                    area + kEps < bestEnvelopeArea) {

                    bestSheet = targetIndex;
                    bestShape = candidateShape;
                    bestPlacement =
                        target.placements.back();
                    bestEnvelopeArea = area;
                }

                target.shapes.resize(oldShapeCount);
                target.placements.resize(oldPlacementCount);
                target.placedArea = oldPlacedArea;
            }

            if (bestSheet == states.size()) {
                allMoved = false;
                break;
            }

            auto& target = states[bestSheet];

            snapshots.push_back({
                bestSheet,
                target.shapes.size(),
                target.placements.size(),
                target.placedArea
            });

            target.shapes.push_back(
                std::move(bestShape)
            );
            target.placements.push_back(bestPlacement);
            target.placedArea +=
                materialArea(instance->part);
        }

        if (!allMoved) {
            for (auto it = snapshots.rbegin();
                 it != snapshots.rend();
                 ++it) {
                auto& target =
                    states[it->sheetIndex];

                target.shapes.resize(
                    it->shapeCount
                );
                target.placements.resize(
                    it->placementCount
                );
                target.placedArea =
                    it->placedArea;
            }
            states[sourceIndex] = std::move(source);
            continue;
        }

        states.erase(
            states.begin() +
            static_cast<std::ptrdiff_t>(sourceIndex)
        );
        if (stats) ++stats->sheetsEliminated;
        changed = true;
    }

    // Refill first: moving already placed details into earlier sheets
    // exposes space that a direct sheet-elimination pass may have missed.
    if (refillExistingSheets(
            states,
            instances,
            sheet,
            options,
            stats
        )) {
        changed = true;
    }

    // Repeatedly try to remove the last sheets. Exchange is deliberately
    // bounded; every successful transaction reduces the primary objective.
    constexpr int kGlobalPasses = 3;
    for (int pass = 0; pass < kGlobalPasses; ++pass) {
        if (stats) ++stats->optimizerPasses;
        bool passChanged = false;

        for (std::size_t sourceIndex = states.size();
             sourceIndex-- > 1;) {

            if (tryExchangeEliminateSheet(
                    sourceIndex,
                    states,
                    instances,
                    sheet,
                    options,
                    stats
                )) {
                changed = true;
                passChanged = true;
            }
        }

        if (refillExistingSheets(
                states,
                instances,
                sheet,
                options,
                stats
            )) {
            changed = true;
            passChanged = true;
        }

        if (!passChanged) break;
    }

    if (!changed) return false;

    result.sheets.clear();
    result.sheets.reserve(states.size());

    double placedArea = 0.0;

    for (auto& state : states) {
        result.sheets.push_back(
            std::move(state.placements)
        );
        placedArea += state.placedArea;
    }

    const double sheetArea =
        std::max(0.0, sheet.width * sheet.height);

    result.utilization =
        (sheetArea > 0.0 && !result.sheets.empty())
            ? placedArea /
              (sheetArea * result.sheets.size())
            : 0.0;

    return true;
}

Result runAttempt(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    std::vector<std::size_t> order,
    std::mt19937& rng,
    NestingStats& stats
) {
    Result result;
    std::vector<SheetState> states;
    states.reserve(16);

    if (order.empty()) return result;

    for (std::size_t position = 0; position < order.size(); ++position) {
        if (shouldStop(options)) {
            for (std::size_t remaining = position;
                 remaining < order.size();
                 ++remaining) {
                result.unplaced.push_back(
                    instances[order[remaining]].id
                );
            }
            break;
        }

        const auto& instance = instances[order[position]];

        std::vector<int> rotations = options.rotations;
        if (rotations.empty()) rotations.push_back(0);
        if (position > 0) std::shuffle(rotations.begin(), rotations.end(), rng);

        // Evaluate every existing sheet, but roll back each trial instead
        // of copying the whole SheetState. Only the winning newly-added shape
        // is retained. This is materially cheaper for large nesting jobs.
        std::size_t bestSheet = states.size();
        PlacedShape bestShape;
        Placement bestPlacement{};
        double bestEnvelopeArea = std::numeric_limits<double>::infinity();
        double bestEnvelopeY = std::numeric_limits<double>::infinity();
        double bestEnvelopeX = std::numeric_limits<double>::infinity();
        bool foundExisting = false;

        for (std::size_t s = 0; s < states.size(); ++s) {
            auto trialRotations = rotations;
            if (s > 0) std::rotate(
                trialRotations.begin(),
                trialRotations.begin() + static_cast<std::ptrdiff_t>(s % trialRotations.size()),
                trialRotations.end()
            );

            auto& state = states[s];
            const std::size_t oldShapeCount = state.shapes.size();
            const std::size_t oldPlacementCount = state.placements.size();
            const double oldPlacedArea = state.placedArea;

            if (!placeOnSheet(
                    instance,
                    sheet,
                    options,
                    state,
                    trialRotations,
                    &stats
                )) {
                continue;
            }

            const auto candidateShape = state.shapes.back();
            Bounds envelope = bounds(candidateShape.outer);
            for (std::size_t i = 0; i + 1 < state.shapes.size(); ++i) {
                const auto b = bounds(state.shapes[i].outer);
                envelope.minX = std::min(envelope.minX, b.minX);
                envelope.minY = std::min(envelope.minY, b.minY);
                envelope.maxX = std::max(envelope.maxX, b.maxX);
                envelope.maxY = std::max(envelope.maxY, b.maxY);
            }
            const double area = envelope.width() * envelope.height();

            if (!foundExisting ||
                area + kEps < bestEnvelopeArea ||
                (std::abs(area - bestEnvelopeArea) <= kEps &&
                 std::tie(envelope.maxY, envelope.maxX) <
                     std::tie(bestEnvelopeY, bestEnvelopeX))) {
                foundExisting = true;
                bestSheet = s;
                bestShape = candidateShape;
                bestPlacement = state.placements.back();
                bestEnvelopeArea = area;
                bestEnvelopeY = envelope.maxY;
                bestEnvelopeX = envelope.maxX;
            }

            state.shapes.resize(oldShapeCount);
            state.placements.resize(oldPlacementCount);
            state.placedArea = oldPlacedArea;
        }

        bool placed = false;
        if (foundExisting) {
            auto& state = states[bestSheet];
            state.shapes.push_back(std::move(bestShape));
            state.placements.push_back(bestPlacement);
            state.placedArea += materialArea(instance.part);
            placed = true;
        } else {
            SheetState state;
            placed = placeOnSheet(
                instance,
                sheet,
                options,
                state,
                rotations,
                &stats
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

    compactResult(
        instances,
        sheet,
        options,
        result,
        &stats
    );

    result.stats = stats;
    return result;
}

} // namespace

Result nest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options
) {
    Result best;
    best.unplaced.reserve(instances.size());
    for (const auto& instance : instances) {
        best.unplaced.push_back(instance.id);
    }
    best.utilization = -1.0;

    if (shouldStop(options)) {
        return best;
    }

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

    const std::size_t iterations = std::max<std::size_t>(1, std::min<std::size_t>(options.iterations, 128u));
    std::mt19937 rng(options.seed);

    for (std::size_t attempt = 0; attempt < iterations; ++attempt) {
        if (shouldStop(options)) break;
        auto attemptOrder = order;

        if (attempt > 0) {
            std::shuffle(attemptOrder.begin(), attemptOrder.end(), rng);
        }

        NestingStats attemptStats;
        auto candidate = runAttempt(
            instances,
            sheet,
            options,
            std::move(attemptOrder),
            rng,
            attemptStats
        );
        candidate.stats = attemptStats;

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
