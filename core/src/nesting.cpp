#include "sheetnest/nesting.hpp"
#include "sheetnest/nfp.hpp"
#include "sheetnest/production_validation.hpp"
#include "sheetnest/spatial_index.hpp"
#include "sheetnest/runtime_tuning.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <limits>
#include <numeric>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace sheetnest {

const char* nestingRecoveryStageName(NestingRecoveryStage stage) {
    switch (stage) {
        case NestingRecoveryStage::None: return "None";
        case NestingRecoveryStage::InitialPlacement: return "InitialPlacement";
        case NestingRecoveryStage::SmallPartRefill: return "SmallPartRefill";
        case NestingRecoveryStage::ResidualRetry: return "ResidualRetry";
        case NestingRecoveryStage::FreshSheetRecovery: return "FreshSheetRecovery";
        case NestingRecoveryStage::Optimizer: return "Optimizer";
        case NestingRecoveryStage::AdaptiveRepair: return "AdaptiveRepair";
    }
    return "None";
}

const char* nestingTelemetryStageName(NestingTelemetryStage stage) {
    switch (stage) {
        case NestingTelemetryStage::None: return "None";
        case NestingTelemetryStage::ExistingSheetSearch: return "ExistingSheetSearch";
        case NestingTelemetryStage::NewSheetSearch: return "NewSheetSearch";
        case NestingTelemetryStage::SmallPartRefill: return "SmallPartRefill";
        case NestingTelemetryStage::ResidualRetry: return "ResidualRetry";
        case NestingTelemetryStage::FreshSheetRecovery: return "FreshSheetRecovery";
        case NestingTelemetryStage::AdaptiveRepair: return "AdaptiveRepair";
        case NestingTelemetryStage::Optimizer: return "Optimizer";
        case NestingTelemetryStage::Finalization: return "Finalization";
    }
    return "None";
}

namespace {

constexpr double kEps = 1e-7;

bool shouldStop(const Options& options) {
    return options.control && options.control->shouldStop();
}

struct PlacedShape {
    Polygon outer;
    std::vector<Polygon> holes;
    Placement placement;
    Bounds outerBounds{};
};

struct SheetState {
    std::vector<PlacedShape> shapes;
    std::vector<Placement> placements;
    double placedArea{};
    SpatialIndex spatialIndex{};
};

struct Candidate {
    double x{};
    double y{};
    double scoreY{};
    double scoreX{};
    double scoreContact{};
    double scoreResidual{};
    double scoreCompactness{};
    double scoreRotation{};
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
    result.outerBounds = bounds(result.outer);
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

double boundsDistance(const Bounds& a, const Bounds& b);

double residualSpaceScore(
    const PlacedShape& shape,
    const SheetState& state,
    const Sheet& sheet,
    double gap
) {
    const double requiredGap = std::max(0.0, gap);
    const double minDim = std::max(
        0.5,
        std::min(shape.outerBounds.width(), shape.outerBounds.height())
    );
    const double horizon = std::clamp(minDim * 2.0, 4.0, 80.0);
    double score = 0.0;

    auto rewardGap = [&](double distance) {
        if (distance < requiredGap - kEps ||
            distance > requiredGap + horizon + kEps) {
            return;
        }
        const double residual = std::max(0.0, distance - requiredGap);
        const double normalized =
            1.0 - std::clamp(residual / horizon, 0.0, 1.0);
        score += normalized * normalized;
    };

    const double edgeDistances[] = {
        shape.outerBounds.minX - sheet.edgeMarginMm,
        shape.outerBounds.minY - sheet.edgeMarginMm,
        sheet.width - sheet.edgeMarginMm - shape.outerBounds.maxX,
        sheet.height - sheet.edgeMarginMm - shape.outerBounds.maxY
    };
    for (const double d : edgeDistances) rewardGap(d);

    const auto nearby = state.spatialIndex.query(
        shape.outerBounds,
        requiredGap + horizon
    );
    for (const auto index : nearby) {
        if (index >= state.shapes.size()) continue;
        const auto& other = state.shapes[index];

        const double dx =
            (shape.outerBounds.maxX < other.outerBounds.minX)
                ? other.outerBounds.minX - shape.outerBounds.maxX
                : (other.outerBounds.maxX < shape.outerBounds.minX)
                    ? shape.outerBounds.minX - other.outerBounds.maxX
                    : 0.0;
        const double dy =
            (shape.outerBounds.maxY < other.outerBounds.minY)
                ? other.outerBounds.minY - shape.outerBounds.maxY
                : (other.outerBounds.maxY < shape.outerBounds.minY)
                    ? shape.outerBounds.minY - other.outerBounds.maxY
                    : 0.0;

        // Reward closing a residual strip only when the perpendicular
        // projection overlaps. This avoids rewarding diagonal separation.
        const bool xAligned =
            shape.outerBounds.maxY >= other.outerBounds.minY - kEps &&
            other.outerBounds.maxY >= shape.outerBounds.minY - kEps;
        const bool yAligned =
            shape.outerBounds.maxX >= other.outerBounds.minX - kEps &&
            other.outerBounds.maxX >= shape.outerBounds.minX - kEps;

        if (xAligned) rewardGap(dx);
        if (yAligned) rewardGap(dy);
    }

    return score;
}

double compactnessScore(
    const PlacedShape& shape,
    const SheetState& state
) {
    if (state.shapes.empty()) return 0.0;

    const auto& b = shape.outerBounds;
    double score = 0.0;
    const auto nearby = state.spatialIndex.query(b, 20.0);
    for (const auto index : nearby) {
        if (index >= state.shapes.size()) continue;
        const auto& other = state.shapes[index];
        const double distance = boundsDistance(b, other.outerBounds);
        score += 1.0 / (1.0 + distance);
    }
    return score;
}

double narrowSpaceScore(
    const PlacedShape& shape,
    const SheetState& state,
    const Sheet& sheet,
    double gap
) {
    const double minDimension = std::max(
        0.5,
        std::min(shape.outerBounds.width(), shape.outerBounds.height())
    );
    const double window = std::clamp(minDimension * 0.75, 0.75, 4.0);
    const double requiredGap = std::max(0.0, gap);
    double score = 0.0;

    const auto nearby = state.spatialIndex.query(
        shape.outerBounds,
        requiredGap + window
    );
    for (const auto index : nearby) {
        if (index >= state.shapes.size()) continue;
        const double distance = boundsDistance(
            shape.outerBounds,
            state.shapes[index].outerBounds
        );
        const double residual = std::max(0.0, distance - requiredGap);
        if (residual <= window + kEps) {
            score += 1.0 + (window - residual) / window;
        }
    }

    // Treat the sheet edge as a usable boundary too. This rewards placements
    // that close narrow strips against the plate perimeter instead of leaving
    // a small unusable corridor between the part and the edge.
    const double margin = std::max(0.0, sheet.edgeMarginMm);
    const double edgeDistances[] = {
        shape.outerBounds.minX - margin,
        shape.outerBounds.minY - margin,
        sheet.width - margin - shape.outerBounds.maxX,
        sheet.height - margin - shape.outerBounds.maxY
    };
    for (const double distance : edgeDistances) {
        if (distance >= -kEps && distance <= requiredGap + window + kEps) {
            const double residual = std::max(0.0, distance - requiredGap);
            score += 0.5 + 0.5 * (window - std::min(window, residual)) / window;
        }
    }

    return score;
}

double boundsDistance(const Bounds& a, const Bounds& b) {
    const double dx =
        (a.maxX < b.minX) ? (b.minX - a.maxX) :
        (b.maxX < a.minX) ? (a.minX - b.maxX) :
        0.0;
    const double dy =
        (a.maxY < b.minY) ? (b.minY - a.maxY) :
        (b.maxY < a.minY) ? (a.minY - b.maxY) :
        0.0;
    return std::hypot(dx, dy);
}

bool conflict(const PlacedShape& a, const PlacedShape& b, double gap) {
    const double requiredGap = std::max(0.0, gap);

    // Broad phase: when the outer AABBs are farther apart than the required
    // clearance, the exact true-shape test cannot possibly report a conflict.
    // Keep the exact collision/clearance checks authoritative for all
    // overlapping or near-touching bounding boxes.
    if (a.outerBounds.maxX + requiredGap < b.outerBounds.minX - kEps ||
        b.outerBounds.maxX + requiredGap < a.outerBounds.minX - kEps ||
        a.outerBounds.maxY + requiredGap < b.outerBounds.minY - kEps ||
        b.outerBounds.maxY + requiredGap < a.outerBounds.minY - kEps) {
        return false;
    }

    if (materialOverlap(a, b)) return true;
    return minBoundaryDistance(a, b) + kEps < requiredGap;
}

Bounds combinedBounds(const std::vector<PlacedShape>& shapes, const PlacedShape& extra) {
    Bounds result = extra.outerBounds;
    for (const auto& shape : shapes) {
        const auto& b = shape.outerBounds;
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
    const auto& b = shape.outerBounds;
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
    const Options& options,
    NestingStats* stats,
    const std::shared_ptr<NestingRunControl>& control
) {
    const Polygon rotatedPart = rotate(part, rotation);
    const auto pb = bounds(rotatedPart);

    // Rotate moving holes once per part/rotation instead of once per placed
    // detail. This removes a repeated geometry transform from the NFP hot path.
    std::vector<Polygon> rotatedHoles;
    rotatedHoles.reserve(holes.size());
    for (const auto& hole : holes) {
        rotatedHoles.push_back(rotate(hole, rotation));
    }

    const double minX = margin - pb.minX;
    const double minY = margin - pb.minY;

    std::vector<Candidate> result;
    result.reserve(64);
    result.push_back({minX, minY, margin, margin});

    const double g = std::max(0.0, gap);
    const double placementMinX = margin - pb.minX;
    const double placementMinY = margin - pb.minY;
    const double placementMaxX = sheetSize.width - margin - pb.maxX;
    const double placementMaxY = sheetSize.height - margin - pb.maxY;
    const double partArea = materialArea(Part{"", part, {}});
    const double smallPartThreshold =
        std::max(
            1.0,
            sheetSize.width * sheetSize.height *
            std::clamp(options.smallPartAreaRatio, 0.001, 0.25)
        );
    const bool smallPart =
        options.enableSmallPartOptimization &&
        partArea <= smallPartThreshold;
    const std::size_t candidateLimit = smallPart
        ? std::max<std::size_t>(
            512,
            std::min<std::size_t>(options.smallPartCandidateBudget, 2048)
        )
        : 512;

    auto addRingCandidates = [&](const Polygon& ring,
                                  const Bounds* knownBounds = nullptr) {
        const auto b = knownBounds ? *knownBounds : bounds(ring);

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

    nfp::NfpRunControl nfpControl;
    auto nfpPairDeadline = std::chrono::steady_clock::time_point::max();
    nfpControl.shouldStop = [control, &nfpPairDeadline]() {
        if (control && control->shouldStop()) return true;
        if (std::chrono::steady_clock::now() >= nfpPairDeadline) return true;
        return false;
    };
    nfpControl.maxInputVertices = options.nfpMaxInputVertices;
    nfpControl.maxConvexPieces = options.nfpMaxConvexPieces;
    nfpControl.maxPairwisePolygons = options.nfpMaxPairwisePolygons;
    nfpControl.maxUnionSegments = options.nfpMaxUnionSegments;
    if (stats) {
        nfpControl.timeoutCount = &stats->nfpTimeouts;
        nfpControl.complexityFallbackCount = &stats->nfpComplexityFallbacks;
        nfpControl.cacheHitCount = &stats->nfpCacheHits;
        nfpControl.cacheMissCount = &stats->nfpCacheMisses;
    }

    for (const auto& placed : sheet.shapes) {
        if (control && control->shouldStop()) break;
        addRingCandidates(placed.outer, &placed.outerBounds);

        for (const auto& rotatedHole : rotatedHoles) {
            addMovingHoleCandidates(
                rotatedHole,
                placed.outer
            );
        }

        // Continuous NFP feasibility boundary. The NFP module owns boundary sampling and candidate budgeting; final true-shape validation remains in placeOnSheet().
        const double forbiddenMinX = placed.outerBounds.minX - pb.maxX - g;
        const double forbiddenMaxX = placed.outerBounds.maxX - pb.minX + g;
        const double forbiddenMinY = placed.outerBounds.minY - pb.maxY - g;
        const double forbiddenMaxY = placed.outerBounds.maxY - pb.minY + g;
        const bool nfpDomainIntersects =
            placementMaxX >= placementMinX &&
            placementMaxY >= placementMinY &&
            forbiddenMaxX >= placementMinX - kEps &&
            forbiddenMinX <= placementMaxX + kEps &&
            forbiddenMaxY >= placementMinY - kEps &&
            forbiddenMinY <= placementMaxY + kEps;

        if (nfpDomainIntersects) {
            if (control && control->shouldStop()) break;
            if (stats) { ++stats->nfpChecks; ++stats->nfpAttempts; }

            const double characteristicSize = std::sqrt(std::max(1.0, partArea));
            const double boundarySpacing = smallPart
                ? std::clamp(options.smallPartBoundarySpacingMm, 0.25, 10.0)
                : std::clamp(std::max(4.0, characteristicSize * 0.08), 4.0, 20.0);
            const std::size_t boundaryBudget = smallPart
                ? std::max<std::size_t>(
                    64,
                    std::min<std::size_t>(
                        std::min<std::size_t>(options.smallPartCandidateBudget, options.nfpCandidateBudget),
                        2048
                    )
                )
                : std::max<std::size_t>(
                    64,
                    std::min<std::size_t>(
                        options.nfpCandidateBudget,
                        part.size() > 256 ? 72 : (part.size() > 128 ? 96 : 128)
                    )
                );

            nfp::SearchOptions searchOptions;
            searchOptions.boundarySpacingMm = boundarySpacing;
            searchOptions.maxCandidates = boundaryBudget;
            searchOptions.includeSheetBoundary = true;
            searchOptions.includeNfpVertices = true;
            searchOptions.includeBoundaryMidpoints = true;

            nfpPairDeadline = options.nfpTimeBudgetMs == 0
                ? std::chrono::steady_clock::time_point::max()
                : std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(options.nfpTimeBudgetMs);
            nfpControl.timeoutRecorded = false;

            const auto search = nfp::searchFeasibleBoundary(
                placed.outer,
                part,
                rotation,
                placementMinX,
                placementMinY,
                placementMaxX,
                placementMaxY,
                g,
                searchOptions,
                &nfpControl
            );

            for (const auto& point : search.points) {
                result.push_back({point.x, point.y, point.y, point.x});
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

    if (result.size() > candidateLimit) {
        if (smallPart) {
            // For dense small-part packing, Y/X order alone can discard the
            // candidates that actually close a narrow residual gap. Evaluate
            // a bounded front pool with the same cheap broad-phase contact
            // score used by placeOnSheet, then keep a deterministic spatial
            // sample from the remainder so distant feasibility segments are
            // still represented.
            const std::size_t contactPool =
                std::min(
                    result.size(),
                    std::max<std::size_t>(
                        candidateLimit,
                        std::min<std::size_t>(
                            result.size(),
                            candidateLimit * 4
                        )
                    )
                );

            for (std::size_t i = 0; i < contactPool; ++i) {
                const auto candidateShape =
                    transformed(
                        Instance{
                            "small-candidate",
                            Part{"small-candidate", part, holes}
                        },
                        rotation,
                        result[i].x,
                        result[i].y
                    );
                result[i].scoreContact = narrowSpaceScore(
                    candidateShape,
                    sheet,
                    sheetSize,
                    gap
                );
            }

            std::stable_sort(
                result.begin(),
                result.begin() +
                    static_cast<std::ptrdiff_t>(contactPool),
                [](const Candidate& a, const Candidate& b) {
                    if (a.scoreContact > b.scoreContact + kEps) return true;
                    if (b.scoreContact > a.scoreContact + kEps) return false;
                    return std::tie(a.scoreY, a.scoreX) <
                           std::tie(b.scoreY, b.scoreX);
                }
            );

            const std::size_t frontBudget =
                std::max<std::size_t>(1, candidateLimit * 7 / 10);
            const std::size_t distributedBudget =
                candidateLimit - frontBudget;

            std::vector<Candidate> diversified;
            diversified.reserve(candidateLimit);

            for (std::size_t i = 0;
                 i < frontBudget && i < contactPool;
                 ++i) {
                diversified.push_back(result[i]);
            }

            if (distributedBudget > 0 &&
                result.size() > frontBudget) {
                const std::size_t tailStart =
                    std::min(frontBudget, result.size() - 1);
                const std::size_t tailSize =
                    result.size() - tailStart;

                for (std::size_t k = 0;
                     k < distributedBudget;
                     ++k) {
                    const std::size_t index =
                        tailStart +
                        (k * (tailSize - 1)) /
                        std::max<std::size_t>(
                            1,
                            distributedBudget - 1
                        );

                    const auto& candidate = result[index];
                    bool duplicate = false;
                    for (const auto& selected : diversified) {
                        if (std::abs(selected.x - candidate.x) <= 1e-6 &&
                            std::abs(selected.y - candidate.y) <= 1e-6) {
                            duplicate = true;
                            break;
                        }
                    }

                    if (!duplicate) diversified.push_back(candidate);
                }
            }

            result = std::move(diversified);
        } else {
            result.resize(candidateLimit);
        }
    }
    return result;
}

bool betterCandidate(const Candidate& a, const Candidate& b) {
    const double aScore =
        a.scoreContact + a.scoreResidual + a.scoreCompactness +
        a.scoreRotation;
    const double bScore =
        b.scoreContact + b.scoreResidual + b.scoreCompactness +
        b.scoreRotation;

    if (aScore > bScore + kEps) return true;
    if (bScore > aScore + kEps) return false;

    if (a.scoreResidual > b.scoreResidual + kEps) return true;
    if (b.scoreResidual > a.scoreResidual + kEps) return false;

    return std::tie(a.scoreY, a.scoreX) < std::tie(b.scoreY, b.scoreX);
}

std::vector<Candidate> fallbackGridCandidates(
    const Polygon& part,
    int rotation,
    const Sheet& sheet,
    double margin,
    std::size_t columns = 32,
    std::size_t rows = 16
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
    // The caller may increase density only when NFP itself timed out, where
    // a narrow feasible corridor is more likely to be under-sampled.
    columns = std::max<std::size_t>(2, columns);
    rows = std::max<std::size_t>(2, rows);
    result.reserve(columns * rows);

    for (std::size_t iy = 0; iy < rows; ++iy) {
        const double ty =
            static_cast<double>(iy) /
            static_cast<double>(rows - 1);

        for (std::size_t ix = 0; ix < columns; ++ix) {
            const double tx =
                static_cast<double>(ix) /
                static_cast<double>(columns - 1);

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

    if (state.spatialIndex.size() != state.shapes.size()) {
        std::vector<Bounds> indexedBounds;
        indexedBounds.reserve(state.shapes.size());
        for (const auto& existing : state.shapes) {
            indexedBounds.push_back(existing.outerBounds);
        }
        state.spatialIndex.rebuild(indexedBounds);
    }

    const double margin = std::max(0.0, sheet.edgeMarginMm);
    const std::size_t nfpTimeoutsBefore = stats ? stats->nfpTimeouts : 0;

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
            if (stats) ++stats->boundsRejections;
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
                 options,
                 stats,
                 options.control)) {
            if (shouldStop(options)) return false;
            if (stats) ++stats->candidateChecks;

            const auto shape =
                transformed(instance, rotation, candidate.x, candidate.y);

            if (!fitsSheet(shape, sheet, sheet.edgeMarginMm)) {
                if (stats) ++stats->boundsRejections;
                continue;
            }

            bool collision = false;
            const auto nearby = state.spatialIndex.query(
                shape.outerBounds,
                options.gapMm
            );
            for (const auto existingIndex : nearby) {
                if (existingIndex >= state.shapes.size()) continue;
                if (stats) ++stats->collisionChecks;
                if (conflict(
                        shape,
                        state.shapes[existingIndex],
                        options.gapMm
                    )) {
                    collision = true;
                    break;
                }
            }
            if (collision) {
                if (stats) ++stats->collisionRejections;
                continue;
            }

            if (stats) ++stats->feasibleCandidates;

            const auto merged = combinedBounds(state.shapes, shape);
            Candidate score = candidate;
            score.scoreY = merged.maxY;
            score.scoreX = merged.maxX;
            const bool smallPartForScore =
                options.enableSmallPartOptimization &&
                materialArea(instance.part) <=
                    std::max(
                        1.0,
                        sheet.width * sheet.height *
                        std::clamp(options.smallPartAreaRatio, 0.001, 0.25)
                    );

            if (smallPartForScore) {
                score.scoreContact = narrowSpaceScore(
                    shape,
                    state,
                    sheet,
                    options.gapMm
                );
            }

            score.scoreResidual =
                options.candidateResidualWeight *
                residualSpaceScore(
                    shape,
                    state,
                    sheet,
                    options.gapMm
                );
            score.scoreCompactness =
                options.candidateCompactnessWeight *
                compactnessScore(shape, state);

            // Prefer rotations that expose the smaller bounding dimension
            // toward the tighter residual axis. This is only a tie-breaker;
            // exact feasibility remains authoritative.
            const double w = shape.outerBounds.width();
            const double h = shape.outerBounds.height();
            const double longAxis = std::max(w, h);
            const double shortAxis = std::max(0.001, std::min(w, h));
            score.scoreRotation =
                options.candidateRotationWeight *
                (longAxis / shortAxis);

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
    // by the exact collision and clearance predicate. When NFP timed out,
    // use a denser deterministic grid to recover narrow feasible corridors.
    // A controller timeout/cancel is still authoritative and never gets
    // bypassed by this recovery path.
    if (!found) {
        const bool nfpTimedOut = stats && stats->nfpTimeouts > nfpTimeoutsBefore;
        if (nfpTimedOut && stats) {
            ++stats->nfpTimeoutFallbacks;
        }
        const std::size_t fallbackColumns = nfpTimedOut ? 48 : 32;
        const std::size_t fallbackRows = nfpTimedOut ? 24 : 16;
        for (int rotation : rotations) {
            if (shouldStop(options)) return false;
            for (const auto& candidate : fallbackGridCandidates(
                     instance.part.outer,
                     rotation,
                     sheet,
                     sheet.edgeMarginMm,
                     fallbackColumns,
                     fallbackRows)) {
                if (shouldStop(options)) return false;
                if (stats) ++stats->candidateChecks;

                const auto shape =
                    transformed(instance, rotation, candidate.x, candidate.y);

                if (!fitsSheet(shape, sheet, sheet.edgeMarginMm)) {
                    if (stats) ++stats->boundsRejections;
                    continue;
                }

                bool collision = false;
                const auto nearby = state.spatialIndex.query(
                    shape.outerBounds,
                    options.gapMm
                );
                for (const auto existingIndex : nearby) {
                    if (existingIndex >= state.shapes.size()) continue;
                    if (stats) ++stats->collisionChecks;
                    if (conflict(
                            shape,
                            state.shapes[existingIndex],
                            options.gapMm
                        )) {
                        collision = true;
                        break;
                    }
                }
                if (collision) {
                    if (stats) ++stats->collisionRejections;
                    continue;
                }

                if (stats) ++stats->feasibleCandidates;

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
    state.spatialIndex.insert(
        state.shapes.size() - 1,
        chosen.outerBounds
    );
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

    std::vector<Bounds> indexedBounds;
    indexedBounds.reserve(placements.size());

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
        indexedBounds.push_back(state.shapes.back().outerBounds);
    }

    state.spatialIndex.rebuild(indexedBounds);
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

    const double usableWidth = std::max(
        0.0,
        sheet.width - 2.0 * std::max(0.0, sheet.edgeMarginMm)
    );
    const double usableHeight = std::max(
        0.0,
        sheet.height - 2.0 * std::max(0.0, sheet.edgeMarginMm)
    );
    const double usableSheetArea = usableWidth * usableHeight;
    const double instanceArea = materialArea(instance.part);

    for (std::size_t targetIndex = 0;
         targetIndex < sheetLimit;
         ++targetIndex) {

        // Safe area lower bound shared by refill/exchange paths. It cannot
        // reject a geometrically feasible arrangement because material area
        // is additive and cannot exceed the usable sheet area.
        if (states[targetIndex].placedArea + instanceArea >
            usableSheetArea + kEps) {
            continue;
        }

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
    const int kPasses = static_cast<int>(std::clamp<std::size_t>(
        options.enableSmallPartOptimization
            ? options.smallPartRefillPasses
            : 3,
        1,
        8
    ));

    for (int pass = 0; pass < kPasses; ++pass) {
        if (shouldStop(options)) break;
        bool passChanged = false;

        for (std::size_t sourceIndex = states.size();
             sourceIndex-- > 1;) {
            if (shouldStop(options)) break;

            auto sourceIds =
                orderedPlacementIds(
                    states[sourceIndex],
                    instances
                );

            // Alternate refill direction: large-first preserves the
            // existing compaction behavior, while small-first passes are
            // specifically aimed at recovering narrow residual gaps that
            // large parts cannot use. Only successful moves are committed.
            if (pass % 2 == 1) {
                std::stable_sort(
                    sourceIds.begin(),
                    sourceIds.end(),
                    [&](const std::string& a, const std::string& b) {
                        const auto* ia = findInstance(instances, a);
                        const auto* ib = findInstance(instances, b);
                        const double aa =
                            ia ? materialArea(ia->part) : 0.0;
                        const double ab =
                            ib ? materialArea(ib->part) : 0.0;

                        if (std::abs(aa - ab) > kEps) {
                            return aa < ab;
                        }

                        return a < b;
                    }
                );
            }

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

                std::size_t bestTargetIndex = sourceIndex;
                bool foundTarget = false;
                double bestEnvelopeArea =
                    std::numeric_limits<double>::infinity();
                double bestMaxY =
                    std::numeric_limits<double>::infinity();
                SheetState bestTargetTrial;

                const double usableWidth = std::max(
                    0.0,
                    sheet.width - 2.0 * std::max(0.0, sheet.edgeMarginMm)
                );
                const double usableHeight = std::max(
                    0.0,
                    sheet.height - 2.0 * std::max(0.0, sheet.edgeMarginMm)
                );
                const double usableSheetArea =
                    usableWidth * usableHeight;
                const double instanceArea = materialArea(instance->part);

                // Build the envelope lower bounds once for this source
                // instance. The states are unchanged while targets are being
                // evaluated, so recomputing the same bounds inside every
                // target check only adds CPU overhead.
                std::vector<Bounds> targetLowerBounds(sourceIndex);
                std::vector<bool> targetHasGeometry(sourceIndex, false);
                for (std::size_t targetIndex = 0;
                     targetIndex < sourceIndex;
                     ++targetIndex) {
                    if (states[targetIndex].shapes.empty()) continue;

                    Bounds lowerBound =
                        states[targetIndex].shapes.front().outerBounds;
                    for (std::size_t i = 1;
                         i < states[targetIndex].shapes.size();
                         ++i) {
                        const auto& b =
                            states[targetIndex].shapes[i].outerBounds;
                        lowerBound.minX =
                            std::min(lowerBound.minX, b.minX);
                        lowerBound.minY =
                            std::min(lowerBound.minY, b.minY);
                        lowerBound.maxX =
                            std::max(lowerBound.maxX, b.maxX);
                        lowerBound.maxY =
                            std::max(lowerBound.maxY, b.maxY);
                    }
                    targetLowerBounds[targetIndex] = lowerBound;
                    targetHasGeometry[targetIndex] = true;
                }

                for (std::size_t targetIndex = 0;
                     targetIndex < sourceIndex;
                     ++targetIndex) {
                    if (shouldStop(options)) break;

                    // Safe area lower bound: if the material already placed
                    // on a sheet plus this instance exceeds the usable sheet
                    // area, no geometric arrangement can fit it there. This
                    // avoids an expensive NFP attempt without rejecting any
                    // physically feasible placement.
                    if (states[targetIndex].placedArea + instanceArea >
                        usableSheetArea + kEps) {
                        continue;
                    }

                    // The refill objective is lexicographic:
                    //   1) minimize the resulting envelope area;
                    //   2) minimize maxY.
                    // The current sheet envelope is a mathematically safe
                    // lower bound for the result after adding the instance:
                    // adding geometry can never shrink min/max coordinates.
                    // Once a better candidate is already found, sheets whose
                    // current envelope is already worse cannot improve it.
                    // This prunes only provably dominated targets and leaves
                    // true-shape feasibility untouched.
                    if (foundTarget && targetHasGeometry[targetIndex]) {
                        const auto& lowerBound =
                            targetLowerBounds[targetIndex];
                        const double lowerBoundArea =
                            lowerBound.width() * lowerBound.height();

                        if (lowerBoundArea > bestEnvelopeArea + kEps ||
                            (std::abs(
                                 lowerBoundArea - bestEnvelopeArea
                             ) <= kEps &&
                             lowerBound.maxY > bestMaxY + kEps)) {
                            continue;
                        }
                    }

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

                    Bounds envelope =
                        targetTrial.shapes.empty()
                            ? Bounds{}
                            : bounds(
                                targetTrial.shapes.front().outer
                            );

                    for (std::size_t i = 1;
                         i < targetTrial.shapes.size();
                         ++i) {
                        const auto& b =
                            targetTrial.shapes[i].outerBounds;
                        envelope.minX =
                            std::min(envelope.minX, b.minX);
                        envelope.minY =
                            std::min(envelope.minY, b.minY);
                        envelope.maxX =
                            std::max(envelope.maxX, b.maxX);
                        envelope.maxY =
                            std::max(envelope.maxY, b.maxY);
                    }

                    const double envelopeArea =
                        envelope.width() * envelope.height();

                    if (!foundTarget ||
                        envelopeArea + kEps < bestEnvelopeArea ||
                        (std::abs(
                             envelopeArea - bestEnvelopeArea
                         ) <= kEps &&
                         envelope.maxY + kEps < bestMaxY)) {
                        bestTargetIndex = targetIndex;
                        bestEnvelopeArea = envelopeArea;
                        bestMaxY = envelope.maxY;
                        bestTargetTrial = std::move(targetTrial);
                        foundTarget = true;
                    }
                }

                if (foundTarget) {
                    auto committed = states;
                    committed[bestTargetIndex] =
                        std::move(bestTargetTrial);
                    erasePlacement(
                        committed[sourceIndex],
                        sourcePos,
                        *instance
                    );
                    states = std::move(committed);

                    changed = true;
                    passChanged = true;
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

void finalizeRecoveryAttribution(Result& result, NestingStats& stats) {
    stats.recovery = {};
    for (const auto& telemetry : result.instanceTelemetry) {
        if (!telemetry.placed) {
            ++stats.recovery.finallyUnplaced;
            continue;
        }
        switch (telemetry.recoveryStage) {
            case NestingRecoveryStage::InitialPlacement:
                ++stats.recovery.initiallyPlaced;
                break;
            case NestingRecoveryStage::SmallPartRefill:
                ++stats.recovery.recoveredBySmallPart;
                break;
            case NestingRecoveryStage::ResidualRetry:
                ++stats.recovery.recoveredByResidualRetry;
                break;
            case NestingRecoveryStage::FreshSheetRecovery:
                ++stats.recovery.recoveredOnNewSheet;
                break;
            case NestingRecoveryStage::Optimizer:
                ++stats.recovery.recoveredByOptimizer;
                break;
            case NestingRecoveryStage::AdaptiveRepair:
                ++stats.recovery.recoveredByAdaptiveRepair;
                break;
            case NestingRecoveryStage::None:
                ++stats.recovery.finallyUnplaced;
                break;
        }
    }
    result.recovery = stats.recovery;
}

double sheetEnvelopeScore(const SheetState& state) {
    if (state.shapes.empty()) {
        return 0.0;
    }

    Bounds envelope = bounds(state.shapes.front().outer);
    for (std::size_t i = 1; i < state.shapes.size(); ++i) {
        const auto b = bounds(state.shapes[i].outer);
        envelope.minX = std::min(envelope.minX, b.minX);
        envelope.minY = std::min(envelope.minY, b.minY);
        envelope.maxX = std::max(envelope.maxX, b.maxX);
        envelope.maxY = std::max(envelope.maxY, b.maxY);
    }

    return envelope.width() * envelope.height();
}

bool betterLocalRepack(
    const SheetState& candidate,
    const SheetState& current
) {
    const double candidateArea = sheetEnvelopeScore(candidate);
    const double currentArea = sheetEnvelopeScore(current);

    if (candidateArea + kEps < currentArea) {
        return true;
    }

    if (std::abs(candidateArea - currentArea) <= kEps) {
        Bounds cb = bounds(candidate.shapes.front().outer);
        Bounds ob = bounds(current.shapes.front().outer);

        for (std::size_t i = 1; i < candidate.shapes.size(); ++i) {
            const auto b = bounds(candidate.shapes[i].outer);
            cb.minX = std::min(cb.minX, b.minX);
            cb.minY = std::min(cb.minY, b.minY);
            cb.maxX = std::max(cb.maxX, b.maxX);
            cb.maxY = std::max(cb.maxY, b.maxY);
        }

        for (std::size_t i = 1; i < current.shapes.size(); ++i) {
            const auto b = bounds(current.shapes[i].outer);
            ob.minX = std::min(ob.minX, b.minX);
            ob.minY = std::min(ob.minY, b.minY);
            ob.maxX = std::max(ob.maxX, b.maxX);
            ob.maxY = std::max(ob.maxY, b.maxY);
        }

        if (cb.maxY + kEps < ob.maxY) return true;
        if (std::abs(cb.maxY - ob.maxY) <= kEps &&
            cb.maxX + kEps < ob.maxX) {
            return true;
        }
    }

    return false;
}

bool localRepack(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    Result& result,
    NestingStats* stats
) {
    if (result.sheets.empty() || shouldStop(options)) {
        return false;
    }

    std::vector<SheetState> states;
    states.reserve(result.sheets.size());

    for (const auto& placements : result.sheets) {
        states.push_back(
            stateFromPlacements(placements, instances)
        );
    }

    bool changed = false;
    constexpr int kMaxRepackPasses = 2;

    for (int pass = 0; pass < kMaxRepackPasses; ++pass) {
        if (shouldStop(options)) break;

        bool passChanged = false;

        for (std::size_t sheetIndex = 0;
             sheetIndex < states.size();
             ++sheetIndex) {
            if (shouldStop(options)) break;

            auto& current = states[sheetIndex];
            if (current.placements.size() < 2) continue;

            std::vector<std::string> ids;
            ids.reserve(current.placements.size());
            for (const auto& placement : current.placements) {
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

            SheetState trial;
            trial.shapes.reserve(current.shapes.size());
            trial.placements.reserve(current.placements.size());

            bool success = true;

            for (const auto& id : ids) {
                if (shouldStop(activeOptions)) {
                    success = false;
                    break;
                }

                const auto* instance = findInstance(instances, id);
                if (!instance ||
                    !placeOnSheet(
                        *instance,
                        sheet,
                        options,
                        trial,
                        effectiveRotations(options),
                        stats
                    )) {
                    success = false;
                    break;
                }
            }

            if (!success ||
                trial.placements.size() != current.placements.size()) {
                continue;
            }

            if (betterLocalRepack(trial, current)) {
                current = std::move(trial);
                if (stats) ++stats->refillMoves;
                passChanged = true;
                changed = true;
            }
        }

        if (!passChanged) break;
        if (stats) ++stats->optimizerPasses;
    }

    if (!changed) return false;

    result.sheets.clear();
    result.sheets.reserve(states.size());

    double placedArea = 0.0;
    for (auto& state : states) {
        result.sheets.push_back(std::move(state.placements));
        placedArea += state.placedArea;
    }

    // Final telemetry reconciliation. Small-part refill and residual recovery
    // can place an instance after its initial attempt marked it unplaced.
    // Recompute the authoritative placed flag from the final sheet list so
    // diagnostics never report a stale "NoFeasiblePosition" for a recovered
    // instance.
    {
        std::unordered_set<std::string> finalPlacedIds;
        for (const auto& sheetPlacements : result.sheets) {
            for (const auto& placement : sheetPlacements) {
                finalPlacedIds.insert(placement.id);
            }
        }

        for (auto& telemetry : result.instanceTelemetry) {
            if (finalPlacedIds.contains(telemetry.instanceId)) {
                telemetry.placed = true;
                telemetry.reason = NestingFailureReason::None;
            }
        }

        result.unplaced.erase(
            std::remove_if(
                result.unplaced.begin(),
                result.unplaced.end(),
                [&](const std::string& id) {
                    return finalPlacedIds.contains(id);
                }
            ),
            result.unplaced.end()
        );
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

            const double usableWidth = std::max(
                0.0,
                sheet.width - 2.0 * std::max(0.0, sheet.edgeMarginMm)
            );
            const double usableHeight = std::max(
                0.0,
                sheet.height - 2.0 * std::max(0.0, sheet.edgeMarginMm)
            );
            const double usableSheetArea = usableWidth * usableHeight;
            const double instanceArea = materialArea(instance->part);

            for (std::size_t targetIndex = 0;
                 targetIndex < sourceIndex;
                 ++targetIndex) {

                auto& target = states[targetIndex];

                if (target.placedArea + instanceArea >
                    usableSheetArea + kEps) {
                    continue;
                }

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
                    candidateShape.outerBounds;

                for (std::size_t i = 0;
                     i + 1 < target.shapes.size();
                     ++i) {

                    const auto& b =
                        target.shapes[i].outerBounds;

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

    result.instanceTelemetry.reserve(instances.size());
    for (const auto& instance : instances) {
        InstanceNestingTelemetry telemetry;
        telemetry.instanceId = instance.id;
        telemetry.unitId = instance.unitId.empty()
            ? instance.id + ":unit-1"
            : instance.unitId;
        telemetry.reason = NestingFailureReason::None;
        telemetry.recoveryStage = NestingRecoveryStage::None;
        telemetry.lastSheetIndex = static_cast<std::size_t>(-1);
        telemetry.stage = NestingTelemetryStage::None;
        result.instanceTelemetry.push_back(std::move(telemetry));
    }

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
        const auto telemetryIndex = order[position];
        const auto telemetryStarted = std::chrono::steady_clock::now();
        const NestingStats statsBeforeInstance = stats;

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
        auto& telemetry = result.instanceTelemetry[telemetryIndex];
        telemetry.stage = foundExisting
            ? NestingTelemetryStage::ExistingSheetSearch
            : NestingTelemetryStage::NewSheetSearch;
        telemetry.stageAttempts++;
        telemetry.lastSheetIndex = foundExisting ? bestSheet : states.size();
        if (foundExisting) {
            auto& state = states[bestSheet];
            state.shapes.push_back(std::move(bestShape));
            state.placements.push_back(bestPlacement);
            state.placedArea += materialArea(instance.part);

            // Keep the broad-phase index synchronized with the committed
            // placement. Without this insertion the next small-part refill
            // would silently fall back to a stale index.
            state.spatialIndex.insert(
                state.shapes.size() - 1,
                state.shapes.back().outerBounds
            );
            placed = true;
            telemetry.recoveryStage = NestingRecoveryStage::InitialPlacement;
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
            if (placed) {
                states.push_back(std::move(state));
                telemetry.recoveryStage = NestingRecoveryStage::InitialPlacement;
            }
        }
        telemetry.candidateChecks += stats.candidateChecks - statsBeforeInstance.candidateChecks;
        telemetry.collisionChecks += stats.collisionChecks - statsBeforeInstance.collisionChecks;
        telemetry.nfpChecks += stats.nfpChecks - statsBeforeInstance.nfpChecks;
        telemetry.nfpAttempts += stats.nfpAttempts - statsBeforeInstance.nfpAttempts;
        telemetry.nfpTimeouts += stats.nfpTimeouts - statsBeforeInstance.nfpTimeouts;
        telemetry.nfpFallbacks += stats.nfpComplexityFallbacks - statsBeforeInstance.nfpComplexityFallbacks;
        telemetry.nfpTimeoutFallbacks += stats.nfpTimeoutFallbacks - statsBeforeInstance.nfpTimeoutFallbacks;
        telemetry.nfpCacheHits += stats.nfpCacheHits - statsBeforeInstance.nfpCacheHits;
        telemetry.nfpCacheMisses += stats.nfpCacheMisses - statsBeforeInstance.nfpCacheMisses;
        telemetry.boundsRejections += stats.boundsRejections - statsBeforeInstance.boundsRejections;
        telemetry.collisionRejections += stats.collisionRejections - statsBeforeInstance.collisionRejections;
        telemetry.feasibleCandidates += stats.feasibleCandidates - statsBeforeInstance.feasibleCandidates;
        telemetry.elapsedMs = static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - telemetryStarted
            ).count()
        );
        telemetry.placed = placed;
        if (!placed) ++telemetry.stageFailures;

        if (!placed) {
            if (options.control && options.control->cancelRequested.load(std::memory_order_relaxed)) {
                telemetry.reason = NestingFailureReason::Cancelled;
            } else if (options.control && options.control->timeoutObserved.load(std::memory_order_relaxed)) {
                telemetry.reason = NestingFailureReason::Timeout;
            } else if (telemetry.nfpTimeouts > 0) {
                telemetry.reason = NestingFailureReason::Timeout;
            } else if (telemetry.nfpFallbacks > 0 && telemetry.candidateChecks == 0) {
                telemetry.reason = NestingFailureReason::InvalidGeometry;
            } else {
                telemetry.reason = NestingFailureReason::NoFeasiblePosition;
            }
            result.unplaced.push_back(instance.id);
        }
    }

    for (auto& telemetry : result.instanceTelemetry) {
        if (telemetry.placed ||
            telemetry.reason != NestingFailureReason::None) {
            continue;
        }

        if (options.control &&
            options.control->cancelRequested.load(std::memory_order_relaxed)) {
            telemetry.reason = NestingFailureReason::Cancelled;
        } else if (options.control &&
                   options.control->timeoutObserved.load(std::memory_order_relaxed)) {
            telemetry.reason = NestingFailureReason::Timeout;
        } else if (std::find(
                       result.unplaced.begin(),
                       result.unplaced.end(),
                       telemetry.instanceId
                   ) != result.unplaced.end()) {
            telemetry.reason = NestingFailureReason::NoFeasiblePosition;
        }
    }

    // Targeted residual-space refill. Only small parts are retried,
    // and only against already-created sheets. No new sheet is opened by
    // this pass, so the pass can only improve packing density.
    if (options.enableSmallPartOptimization &&
        !result.unplaced.empty() &&
        !states.empty() &&
        options.smallPartRefillPasses > 0) {

        const double sheetArea =
            std::max(1.0, sheet.width * sheet.height);
        const double maxSmallArea =
            sheetArea *
            std::clamp(
                options.smallPartAreaRatio,
                0.001,
                0.25
            );

        std::vector<std::string> remaining = result.unplaced;

        for (std::size_t pass = 0;
             pass < options.smallPartRefillPasses &&
             !remaining.empty();
             ++pass) {
            if (shouldStop(options)) break;

            std::stable_sort(
                remaining.begin(),
                remaining.end(),
                [&](const std::string& a, const std::string& b) {
                    const auto* ia = findInstance(instances, a);
                    const auto* ib = findInstance(instances, b);
                    const double aa =
                        ia ? materialArea(ia->part) : 0.0;
                    const double ab =
                        ib ? materialArea(ib->part) : 0.0;
                    if (std::abs(aa - ab) > kEps) {
                        return aa < ab;
                    }
                    return a < b;
                }
            );

            std::vector<std::string> nextRemaining;
            nextRemaining.reserve(remaining.size());

            for (const auto& id : remaining) {
                const auto* instance =
                    findInstance(instances, id);

                if (!instance ||
                    materialArea(instance->part) > maxSmallArea) {
                    nextRemaining.push_back(id);
                    continue;
                }

                if (shouldStop(options)) {
                    nextRemaining.push_back(id);
                    continue;
                }

                const auto refillStatsBefore = stats;
                const auto telemetryIt = std::find_if(
                    result.instanceTelemetry.begin(),
                    result.instanceTelemetry.end(),
                    [&](const InstanceNestingTelemetry& t) {
                        return t.instanceId == id;
                    }
                );
                if (telemetryIt != result.instanceTelemetry.end()) {
                    telemetryIt->stage = NestingTelemetryStage::SmallPartRefill;
                    telemetryIt->stageAttempts++;
                }

                const bool refillPlaced = tryPlaceOnExistingSheets(
                        *instance,
                        states,
                        states.size(),
                        sheet,
                        options,
                        &stats
                    );
                if (refillPlaced) {
                    ++stats.refillMoves;
                    telemetryIt->recoveryStage = NestingRecoveryStage::SmallPartRefill;
                    telemetryIt->placed = true;
                    telemetryIt->reason = NestingFailureReason::None;
                } else {
                    nextRemaining.push_back(id);
                }

                if (telemetryIt != result.instanceTelemetry.end()) {
                    telemetryIt->candidateChecks += stats.candidateChecks - refillStatsBefore.candidateChecks;
                    telemetryIt->collisionChecks += stats.collisionChecks - refillStatsBefore.collisionChecks;
                    telemetryIt->nfpChecks += stats.nfpChecks - refillStatsBefore.nfpChecks;
                    telemetryIt->nfpAttempts += stats.nfpAttempts - refillStatsBefore.nfpAttempts;
                    telemetryIt->nfpTimeouts += stats.nfpTimeouts - refillStatsBefore.nfpTimeouts;
                    telemetryIt->nfpFallbacks += stats.nfpComplexityFallbacks - refillStatsBefore.nfpComplexityFallbacks;
                    telemetryIt->nfpTimeoutFallbacks += stats.nfpTimeoutFallbacks - refillStatsBefore.nfpTimeoutFallbacks;
                    telemetryIt->nfpCacheHits += stats.nfpCacheHits - refillStatsBefore.nfpCacheHits;
                    telemetryIt->nfpCacheMisses += stats.nfpCacheMisses - refillStatsBefore.nfpCacheMisses;
                    telemetryIt->boundsRejections += stats.boundsRejections - refillStatsBefore.boundsRejections;
                    telemetryIt->collisionRejections += stats.collisionRejections - refillStatsBefore.collisionRejections;
                    telemetryIt->feasibleCandidates += stats.feasibleCandidates - refillStatsBefore.feasibleCandidates;
                    telemetryIt->lastSheetIndex = states.empty() ? static_cast<std::size_t>(-1) : states.size() - 1;
                    if (telemetryIt->placed) {
                        telemetryIt->reason = NestingFailureReason::None;
                    }
                }
            }

            remaining = std::move(nextRemaining);
        }

        result.unplaced = std::move(remaining);
    }

    // Bounded new-sheet recovery: residual instances may have failed because
    // the original ordering exhausted the useful candidate search budget.
    // Give each residual instance one deterministic attempt on a fresh sheet.
    // This is deliberately separate from the residual-space refill so it can
    // never create an unbounded sheet-generation loop.
    if (!result.unplaced.empty() && !shouldStop(options)) {
        // Bounded residual retry is applied to every residual part, not only
        // small parts. This closes the common false-failure case where a
        // greedy ordering misses a valid position on an already created sheet.
        // Each pass is transactional at the sheet level and never bypasses
        // exact true-shape validation.
        const std::size_t retryPasses =
            std::clamp<std::size_t>(options.residualRetryPasses, 1, 6);
        for (std::size_t pass = 0; pass < retryPasses && !result.unplaced.empty(); ++pass) {
            if (shouldStop(options)) break;

            std::vector<std::string> next;
            next.reserve(result.unplaced.size());

            for (const auto& id : result.unplaced) {
                const auto* instance = findInstance(instances, id);
                if (!instance || shouldStop(options)) {
                    next.push_back(id);
                    continue;
                }

                bool recoveredOnExisting = false;
                const auto retryStatsBefore = stats;
                const auto retryTelemetryIt = std::find_if(
                    result.instanceTelemetry.begin(),
                    result.instanceTelemetry.end(),
                    [&](const InstanceNestingTelemetry& t) {
                        return t.instanceId == id;
                    }
                );
                if (retryTelemetryIt != result.instanceTelemetry.end()) {
                    retryTelemetryIt->stage = NestingTelemetryStage::ResidualRetry;
                    retryTelemetryIt->stageAttempts++;
                }
                // Try every existing sheet in a different order on each pass.
                // This deliberately revisits sheets rejected by the initial
                // greedy ordering.
                const std::size_t count = states.size();
                if (count > 0) {
                    std::vector<std::size_t> sheetOrder(count);
                    std::iota(sheetOrder.begin(), sheetOrder.end(), 0);
                    if (pass % 2 == 1) std::reverse(sheetOrder.begin(), sheetOrder.end());

                    for (const auto sheetIndex : sheetOrder) {
                        if (shouldStop(options)) break;
                        SheetState trial = states[sheetIndex];
                        if (!placeOnSheet(
                                *instance,
                                sheet,
                                options,
                                trial,
                                effectiveRotations(options),
                                &stats
                            )) {
                            continue;
                        }
                        states[sheetIndex] = std::move(trial);
                        recoveredOnExisting = true;
                        ++stats.refillMoves;
                        retryTelemetryIt->recoveryStage = NestingRecoveryStage::ResidualRetry;
                        retryTelemetryIt->placed = true;
                        retryTelemetryIt->reason = NestingFailureReason::None;
                        break;
                    }
                }

                if (!recoveredOnExisting) next.push_back(id);

                if (retryTelemetryIt != result.instanceTelemetry.end()) {
                    retryTelemetryIt->candidateChecks += stats.candidateChecks - retryStatsBefore.candidateChecks;
                    retryTelemetryIt->collisionChecks += stats.collisionChecks - retryStatsBefore.collisionChecks;
                    retryTelemetryIt->nfpChecks += stats.nfpChecks - retryStatsBefore.nfpChecks;
                    retryTelemetryIt->nfpAttempts += stats.nfpAttempts - retryStatsBefore.nfpAttempts;
                    retryTelemetryIt->nfpTimeouts += stats.nfpTimeouts - retryStatsBefore.nfpTimeouts;
                    retryTelemetryIt->nfpFallbacks += stats.nfpComplexityFallbacks - retryStatsBefore.nfpComplexityFallbacks;
                    retryTelemetryIt->nfpTimeoutFallbacks += stats.nfpTimeoutFallbacks - retryStatsBefore.nfpTimeoutFallbacks;
                    retryTelemetryIt->nfpCacheHits += stats.nfpCacheHits - retryStatsBefore.nfpCacheHits;
                    retryTelemetryIt->nfpCacheMisses += stats.nfpCacheMisses - retryStatsBefore.nfpCacheMisses;
                    retryTelemetryIt->boundsRejections += stats.boundsRejections - retryStatsBefore.boundsRejections;
                    retryTelemetryIt->collisionRejections += stats.collisionRejections - retryStatsBefore.collisionRejections;
                    retryTelemetryIt->feasibleCandidates += stats.feasibleCandidates - retryStatsBefore.feasibleCandidates;
                    retryTelemetryIt->lastSheetIndex = states.empty() ? static_cast<std::size_t>(-1) : states.size() - 1;
                    if (!recoveredOnExisting) ++retryTelemetryIt->stageFailures;
                }
            }

            result.unplaced = std::move(next);
        }

        std::vector<std::string> recovered;
        recovered.reserve(result.unplaced.size());

        for (const auto& id : result.unplaced) {
            if (shouldStop(options)) break;

            const auto* instance = findInstance(instances, id);
            if (!instance) continue;

            SheetState recoveryState;
            const auto before = stats;
            const auto telemetryIt = std::find_if(
                result.instanceTelemetry.begin(),
                result.instanceTelemetry.end(),
                [&](const InstanceNestingTelemetry& telemetry) {
                    return telemetry.instanceId == id;
                }
            );
            if (telemetryIt != result.instanceTelemetry.end()) {
                telemetryIt->stage = NestingTelemetryStage::FreshSheetRecovery;
                telemetryIt->stageAttempts++;
                telemetryIt->lastSheetIndex = states.size();
            }
            const bool placed = placeOnSheet(
                *instance,
                sheet,
                options,
                recoveryState,
                effectiveRotations(options),
                &stats
            );

            const auto recoveryTelemetryIt = std::find_if(
                result.instanceTelemetry.begin(),
                result.instanceTelemetry.end(),
                [&](const InstanceNestingTelemetry& telemetry) {
                    return telemetry.instanceId == id;
                }
            );

            if (recoveryTelemetryIt != result.instanceTelemetry.end()) {
                recoveryTelemetryIt->candidateChecks +=
                    stats.candidateChecks - before.candidateChecks;
                recoveryTelemetryIt->collisionChecks +=
                    stats.collisionChecks - before.collisionChecks;
                recoveryTelemetryIt->nfpChecks +=
                    stats.nfpChecks - before.nfpChecks;
                recoveryTelemetryIt->nfpAttempts +=
                    stats.nfpAttempts - before.nfpAttempts;
                recoveryTelemetryIt->nfpTimeouts +=
                    stats.nfpTimeouts - before.nfpTimeouts;
                recoveryTelemetryIt->nfpFallbacks +=
                    stats.nfpComplexityFallbacks -
                    before.nfpComplexityFallbacks;
                recoveryTelemetryIt->nfpCacheHits +=
                    stats.nfpCacheHits - before.nfpCacheHits;
                recoveryTelemetryIt->nfpCacheMisses +=
                    stats.nfpCacheMisses - before.nfpCacheMisses;
                recoveryTelemetryIt->boundsRejections +=
                    stats.boundsRejections - before.boundsRejections;
                recoveryTelemetryIt->collisionRejections +=
                    stats.collisionRejections -
                    before.collisionRejections;
                recoveryTelemetryIt->feasibleCandidates +=
                    stats.feasibleCandidates -
                    before.feasibleCandidates;
            }

            if (placed) {
                states.push_back(std::move(recoveryState));
                recovered.push_back(id);

                if (telemetryIt != result.instanceTelemetry.end()) {
                    telemetryIt->placed = true;
                    telemetryIt->reason = NestingFailureReason::None;
                    telemetryIt->stage = NestingTelemetryStage::FreshSheetRecovery;
                    telemetryIt->recoveryStage = NestingRecoveryStage::FreshSheetRecovery;
                }
            }
        }

        if (!recovered.empty()) {
            result.unplaced.erase(
                std::remove_if(
                    result.unplaced.begin(),
                    result.unplaced.end(),
                    [&](const std::string& id) {
                        return std::find(
                            recovered.begin(),
                            recovered.end(),
                            id
                        ) != recovered.end();
                    }
                ),
                result.unplaced.end()
            );
        }
    }

    result.sheets.reserve(states.size());
    double placedArea = 0.0;
    std::unordered_set<std::string> finalPlacedIds;
    for (const auto& state : states) {
        for (const auto& placement : state.placements) {
            finalPlacedIds.insert(placement.id);
        }
    }

    result.unplaced.erase(
        std::remove_if(
            result.unplaced.begin(), result.unplaced.end(),
            [&](const std::string& id) {
                return finalPlacedIds.contains(id);
            }),
        result.unplaced.end());

    for (auto& telemetry : result.instanceTelemetry) {
        if (finalPlacedIds.contains(telemetry.instanceId)) {
            telemetry.placed = true;
            telemetry.reason = NestingFailureReason::None;
        }
    }

    for (auto& state : states) {
        result.sheets.push_back(std::move(state.placements));
        placedArea += state.placedArea;
    }

    const double sheetArea = std::max(0.0, sheet.width * sheet.height);
    result.utilization =
        (sheetArea > 0.0 && !result.sheets.empty())
            ? placedArea / (sheetArea * result.sheets.size())
            : 0.0;

    if (options.enableOptimizer) {
        compactResult(
            instances,
            sheet,
            options,
            result,
            &stats
        );
    }

    finalizeRecoveryAttribution(result, stats);
    result.stats = stats;
    return result;
}

} // namespace

Result nest(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options
) {
    const auto tuning = tuneNestingOptions(instances, sheet, options);
    const Options activeOptions = tuning.options;

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

    const std::size_t iterations = std::max<std::size_t>(1, std::min<std::size_t>(activeOptions.iterations, 128u));
    std::mt19937 rng(activeOptions.seed);

    for (std::size_t attempt = 0; attempt < iterations; ++attempt) {
        if (shouldStop(options)) break;
        auto attemptOrder = order;

        if (attempt > 0) {
            if (attempt % 3 == 1) {
                // Small-first restart: explicitly targets narrow residual
                // spaces that can be missed when large parts monopolize the
                // candidate order.
                std::stable_sort(
                    attemptOrder.begin(),
                    attemptOrder.end(),
                    [&](std::size_t a, std::size_t b) {
                        const double areaA =
                            materialArea(instances[a].part);
                        const double areaB =
                            materialArea(instances[b].part);

                        if (std::abs(areaA - areaB) > kEps) {
                            return areaA < areaB;
                        }

                        return instances[a].id < instances[b].id;
                    }
                );
            } else if (attempt % 3 == 2) {
                // Large-dimension-first restart: different from pure area
                // ordering and useful for long/narrow details.
                std::stable_sort(
                    attemptOrder.begin(),
                    attemptOrder.end(),
                    [&](std::size_t a, std::size_t b) {
                        const auto ba =
                            bounds(instances[a].part.outer);
                        const auto bb =
                            bounds(instances[b].part.outer);

                        const double da =
                            std::max(ba.width(), ba.height());
                        const double db =
                            std::max(bb.width(), bb.height());

                        if (std::abs(da - db) > kEps) {
                            return da > db;
                        }

                        const double areaA =
                            materialArea(instances[a].part);
                        const double areaB =
                            materialArea(instances[b].part);

                        if (std::abs(areaA - areaB) > kEps) {
                            return areaA > areaB;
                        }

                        return instances[a].id < instances[b].id;
                    }
                );
            } else {
                std::shuffle(
                    attemptOrder.begin(),
                    attemptOrder.end(),
                    rng
                );
            }
        }

        NestingStats attemptStats;
        auto candidate = runAttempt(
            instances,
            sheet,
            activeOptions,
            std::move(attemptOrder),
            rng,
            attemptStats
        );
        candidate.stats = attemptStats;

        if (best.utilization < 0.0 || betterResult(candidate, best)) {
            best = std::move(candidate);
        }

        // A complete placement is authoritative: never allow a later
        // heuristic attempt with more unplaced parts to replace it merely
        // because its utilization happens to be higher.
        if (best.unplaced.empty() && !candidate.unplaced.empty()) {
            // Keep the complete result already selected.
        }

        // A feasible single-sheet result with every requested instance is a
        // hard lower bound on the primary objective, so further restarts can
        // only improve secondary utilization.
        if (best.unplaced.empty() && best.sheets.size() == 1) {
            // Keep searching when explicitly requested; the utilization
            // comparison still decides whether another restart is better.
        }
    }

    if (activeOptions.enableProductionValidation) {
        const auto validation = validateProductionResult(
            instances,
            sheet,
            options,
            best
        );

        best.productionValidated = true;
        best.productionValid = validation.valid;
        best.productionIssueCount = validation.issues.size();
    }

    return best;
}

bool optimizeNestingResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    Result& result
) {
    NestingStats stats = result.stats;
    bool changed = false;

    if (shouldStop(options)) {
        result.stats = stats;
        return false;
    }

    changed |= compactResult(
        instances,
        sheet,
        options,
        result,
        &stats
    );

    if (!shouldStop(options)) {
        changed |= localRepack(
            instances,
            sheet,
            options,
            result,
            &stats
        );
    }

    if (!shouldStop(options)) {
        changed |= compactResult(
            instances,
            sheet,
            options,
            result,
            &stats
        );
    }

    if (!shouldStop(options)) {
        changed |= localRepack(
            instances,
            sheet,
            options,
            result,
            &stats
        );
    }

    if (!shouldStop(options)) {
        changed |= compactResult(
            instances,
            sheet,
            options,
            result,
            &stats
        );
    }

    stats.optimizerPasses += 1;
    result.stats = stats;
    return changed;
}

bool adaptiveDestroyAndRepairResult(
    const std::vector<Instance>& instances,
    const Sheet& sheet,
    const Options& options,
    const std::vector<std::string>& seedIds,
    Result& result,
    std::vector<std::string>* extractedIdsOut
) {
    if (result.sheets.empty() ||
        seedIds.empty() ||
        shouldStop(options)) {
        return false;
    }

    std::unordered_set<std::string> seedSet(
        seedIds.begin(),
        seedIds.end()
    );

    std::vector<SheetState> baseStates;
    baseStates.reserve(result.sheets.size());
    for (const auto& placements : result.sheets) {
        baseStates.push_back(
            stateFromPlacements(placements, instances)
        );
    }

    std::vector<std::size_t> affectedSheets;
    affectedSheets.reserve(baseStates.size());

    // Start with the validator's concrete conflict IDs.
    for (std::size_t sheetIndex = 0;
         sheetIndex < baseStates.size();
         ++sheetIndex) {
        bool affected = false;
        for (const auto& placement : baseStates[sheetIndex].placements) {
            if (seedSet.contains(placement.id)) {
                affected = true;
                break;
            }
        }
        if (affected) {
            affectedSheets.push_back(sheetIndex);
        }
    }

    if (affectedSheets.empty()) {
        return false;
    }

    const double gap =
        std::max(0.0, options.gapMm);

    std::unordered_set<std::string> extractedIds;
    std::vector<std::vector<std::size_t>> extractedIndices(
        baseStates.size()
    );

    // Build a small local conflict neighborhood on each affected sheet.
    // Direct geometric conflicts are always included first. Then include
    // nearby blockers by true-shape boundary distance, with a hard cap so a
    // local repair cannot accidentally become a full-sheet repack.
    for (const auto sheetIndex : affectedSheets) {
        auto& state = baseStates[sheetIndex];
        auto& selected = extractedIndices[sheetIndex];

        for (std::size_t i = 0; i < state.placements.size(); ++i) {
            if (seedSet.contains(state.placements[i].id)) {
                selected.push_back(i);
                extractedIds.insert(state.placements[i].id);
            }
        }

        if (selected.empty()) continue;

        auto regionBounds = bounds(
            state.shapes[selected.front()].outer
        );
        for (std::size_t k = 1; k < selected.size(); ++k) {
            const auto b =
                bounds(state.shapes[selected[k]].outer);
            regionBounds.minX =
                std::min(regionBounds.minX, b.minX);
            regionBounds.minY =
                std::min(regionBounds.minY, b.minY);
            regionBounds.maxX =
                std::max(regionBounds.maxX, b.maxX);
            regionBounds.maxY =
                std::max(regionBounds.maxY, b.maxY);
        }

        double maxDimension = std::max(
            regionBounds.width(),
            regionBounds.height()
        );

        const double localRadius = std::clamp(
            std::max({
                4.0 * gap,
                5.0,
                0.25 * maxDimension
            }),
            5.0,
            250.0
        );

        const std::size_t maxNeighbors =
            std::max<std::size_t>(
                0,
                options.adaptiveRepairMaxNeighbors
            );

        std::vector<bool> selectedFlags(
            state.placements.size(),
            false
        );
        for (const auto index : selected) {
            if (index < selectedFlags.size()) {
                selectedFlags[index] = true;
            }
        }

        struct Nearby {
            std::size_t index{};
            double distance{};
            bool directConflict{};
        };

        std::vector<Nearby> nearby;
        for (std::size_t i = 0;
             i < state.placements.size();
             ++i) {
            if (selectedFlags[i]) {
                continue;
            }

            const auto& candidateShape = state.shapes[i];
            double bestDistance =
                std::numeric_limits<double>::infinity();
            bool directConflict = false;

            for (const auto selectedIndex : selected) {
                const auto& selectedShape =
                    state.shapes[selectedIndex];

                // Cheap AABB-radius rejection avoids the O(V^2) true-shape
                // boundary walk for distant placements.
                if (boundsDistance(
                        candidateShape.outerBounds,
                        selectedShape.outerBounds
                    ) > localRadius + kEps) {
                    continue;
                }

                const bool materialConflict =
                    materialOverlap(
                        candidateShape,
                        selectedShape
                    );

                const double exactDistance =
                    minBoundaryDistance(
                        candidateShape,
                        selectedShape
                    );

                const bool pairConflict =
                    materialConflict ||
                    exactDistance + kEps < gap;

                if (pairConflict) {
                    directConflict = true;
                    bestDistance = 0.0;
                    break;
                }

                bestDistance = std::min(
                    bestDistance,
                    exactDistance
                );
            }

            if (directConflict ||
                bestDistance <= localRadius + kEps) {
                nearby.push_back({
                    i,
                    bestDistance,
                    directConflict
                });
            }
        }

        std::sort(
            nearby.begin(),
            nearby.end(),
            [](const Nearby& a, const Nearby& b) {
                if (a.directConflict != b.directConflict) {
                    return a.directConflict > b.directConflict;
                }
                return a.distance < b.distance;
            }
        );

        std::size_t added = 0;
        for (const auto& item : nearby) {
            if (added >= maxNeighbors) break;
            selected.push_back(item.index);
            if (item.index < selectedFlags.size()) {
                selectedFlags[item.index] = true;
            }
            extractedIds.insert(
                state.placements[item.index].id
            );
            ++added;
        }
    }

    if (extractedIds.empty()) {
        return false;
    }

    // The group is intentionally compact: we do not touch unrelated sheets
    // or placements outside the affected local neighborhoods.
    std::vector<SheetState> strippedStates = baseStates;
    for (const auto sheetIndex : affectedSheets) {
        auto& state = strippedStates[sheetIndex];

        std::vector<std::size_t> indices =
            extractedIndices[sheetIndex];

        std::sort(
            indices.rbegin(),
            indices.rend()
        );

        for (const auto index : indices) {
            if (index >= state.placements.size()) continue;

            const auto* instance =
                findInstance(
                    instances,
                    state.placements[index].id
                );

            if (!instance) continue;
            erasePlacement(state, index, *instance);
        }
    }

    std::vector<const Instance*> groupInstances;
    groupInstances.reserve(extractedIds.size());
    for (const auto& id : extractedIds) {
        const auto* instance =
            findInstance(instances, id);
        if (instance) {
            groupInstances.push_back(instance);
        }
    }

    if (groupInstances.empty()) {
        return false;
    }

    auto makeOrder = [&](std::size_t attempt) {
        std::vector<const Instance*> ordered =
            groupInstances;

        std::sort(
            ordered.begin(),
            ordered.end(),
            [&](const Instance* a, const Instance* b) {
                const double aa = materialArea(a->part);
                const double ab = materialArea(b->part);

                if (attempt % 3 == 1) {
                    if (std::abs(aa - ab) > kEps) {
                        return aa < ab;
                    }
                } else if (attempt % 3 == 2) {
                    const auto ba = bounds(a->part.outer);
                    const auto bb = bounds(b->part.outer);
                    const double da =
                        std::max(ba.width(), ba.height());
                    const double db =
                        std::max(bb.width(), bb.height());

                    if (std::abs(da - db) > kEps) {
                        return da > db;
                    }
                } else if (std::abs(aa - ab) > kEps) {
                    return aa > ab;
                }

                return a->id < b->id;
            }
        );

        if (attempt >= 3) {
            std::mt19937 localRng(
                options.seed +
                static_cast<std::uint32_t>(
                    0x9E3779B9u *
                    static_cast<std::uint32_t>(attempt + 1)
                )
            );
            std::shuffle(
                ordered.begin(),
                ordered.end(),
                localRng
            );
        }

        return ordered;
    };

    // Keep the best state of the previous repair round as the baseline
    // for the next round. Attempts inside one round remain independent so
    // they can explore different orders, but completed rounds are now truly
    // sequential instead of all restarting from the same stripped layout.
    std::vector<SheetState> roundBaselineStates = strippedStates;
    std::vector<SheetState> bestStates = strippedStates;
    bool foundComplete = false;
    double bestLocalScore =
        std::numeric_limits<double>::infinity();

    const std::size_t attemptsPerRound =
        std::clamp<std::size_t>(
            std::max<std::size_t>(
                1,
                options.adaptiveRepairAttempts
            ),
            1,
            8
        );
    const std::size_t repairRounds =
        std::clamp<std::size_t>(
            std::max<std::size_t>(
                1,
                options.adaptiveRepairRounds
            ),
            1,
            8
        );
    const std::size_t attempts =
        std::min<std::size_t>(
            32,
            attemptsPerRound * repairRounds
        );
    NestingStats adaptiveStats = result.stats;

    for (std::size_t round = 0;
         round < repairRounds && round * attemptsPerRound < attempts;
         ++round) {
        if (shouldStop(options)) break;

        bool roundFoundComplete = false;
        double roundBestScore =
            std::numeric_limits<double>::infinity();
        std::vector<SheetState> roundBestStates;

        const std::size_t roundAttemptCount =
            std::min(
                attemptsPerRound,
                attempts - round * attemptsPerRound
            );

        for (std::size_t attemptInRound = 0;
             attemptInRound < roundAttemptCount;
             ++attemptInRound) {
            if (shouldStop(options)) break;

            const std::size_t attempt =
                round * attemptsPerRound + attemptInRound;

            // Every attempt in this round starts from the same round baseline;
            // the winning baseline is committed only after the round finishes.
            auto trialStates = roundBaselineStates;
            const auto ordered = makeOrder(attempt);
            bool success = true;

            for (const auto* instance : ordered) {
                if (shouldStop(options)) {
                    success = false;
                    break;
                }

                bool placed = false;
                std::size_t bestSheetIndex =
                    trialStates.size();
                SheetState bestSheetState;
                double bestSheetScore =
                    std::numeric_limits<double>::infinity();

                for (const auto sheetIndex : affectedSheets) {
                    SheetState trial =
                        trialStates[sheetIndex];

                    if (!placeOnSheet(
                            *instance,
                            sheet,
                            options,
                            trial,
                            effectiveRotations(options),
                            &adaptiveStats
                        )) {
                        continue;
                    }

                    const double score =
                        sheetEnvelopeScore(trial);

                    if (!placed ||
                        score + kEps < bestSheetScore) {
                        placed = true;
                        bestSheetIndex = sheetIndex;
                        bestSheetState = std::move(trial);
                        bestSheetScore = score;
                    }
                }

                if (!placed) {
                    success = false;
                    break;
                }

                trialStates[bestSheetIndex] =
                    std::move(bestSheetState);
            }

            if (!success) continue;

            double localScore = 0.0;
            for (const auto sheetIndex : affectedSheets) {
                localScore +=
                    sheetEnvelopeScore(trialStates[sheetIndex]);
            }

            if (!roundFoundComplete ||
                localScore + kEps < roundBestScore) {
                roundFoundComplete = true;
                roundBestScore = localScore;
                roundBestStates = std::move(trialStates);
            }
        }

        // A failed round cannot provide a meaningful baseline for the next
        // round, so stop without replacing the best completed result.
        if (!roundFoundComplete) {
            break;
        }

        // This is the key sequential repair step: the best completed local
        // repair becomes the starting state of the next adaptive round.
        roundBaselineStates = roundBestStates;

        if (!foundComplete ||
            roundBestScore + kEps < bestLocalScore) {
            foundComplete = true;
            bestLocalScore = roundBestScore;
            bestStates = roundBaselineStates;
        }
    }
    if (!foundComplete) {
        return false;
    }

    Result bestResult = result;
    bestResult.sheets.clear();
    bestResult.sheets.reserve(bestStates.size());

    double placedArea = 0.0;
    for (const auto& state : bestStates) {
        bestResult.sheets.push_back(state.placements);
        placedArea += state.placedArea;
    }

    const double sheetArea =
        std::max(0.0, sheet.width * sheet.height);
    bestResult.utilization =
        (sheetArea > 0.0 && !bestResult.sheets.empty())
            ? placedArea / (sheetArea * bestResult.sheets.size())
            : 0.0;
    bestResult.stats = adaptiveStats;
    ++bestResult.stats.refillMoves;
    ++bestResult.stats.optimizerPasses;

    if (extractedIdsOut) {
        extractedIdsOut->assign(
            extractedIds.begin(),
            extractedIds.end()
        );
        std::sort(
            extractedIdsOut->begin(),
            extractedIdsOut->end()
        );
    }

    // Preserve the original unplaced set: this operation only rearranges
    // already-placed instances. The Production Validator decides whether the
    // resulting layout is safe.
    bestResult.productionValidated = false;
    bestResult.productionValid = false;
    bestResult.productionIssueCount = 0;

    result = std::move(bestResult);
    return true;
}

} // namespace sheetnest