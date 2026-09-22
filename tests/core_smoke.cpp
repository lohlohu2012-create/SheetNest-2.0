#include "sheetnest/dxf.hpp"
#include "sheetnest/dxf_export.hpp"
#include "sheetnest/benchmark.hpp"
#include "sheetnest/diagnostics.hpp"
#include "sheetnest/dxf_model.hpp"
#include "sheetnest/geometry.hpp"
#include "sheetnest/nesting.hpp"
#include "sheetnest/nfp.hpp"
#include "sheetnest/parallel_nesting.hpp"
#include "sheetnest/production_validation.hpp"
#include "sheetnest/spatial_index.hpp"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <atomic>

using namespace sheetnest;

namespace {

const char* kLayerSeparatedDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LINE
8
A
10
0
20
0
11
10
21
0
0
LINE
8
A
10
10
20
0
11
10
21
10
0
LINE
8
A
10
10
20
10
11
0
21
10
0
LINE
8
A
10
0
20
10
11
0
21
0
0
LINE
8
B
10
10
20
10
11
20
21
10
0
LINE
8
B
10
20
20
10
11
20
21
20
0
LINE
8
B
10
20
20
20
11
10
21
20
0
LINE
8
B
10
10
20
20
11
10
21
10
0
ENDSEC
0
EOF
)DXF";



const char* kLineArcClosedDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LINE
8
PARTS
10
0
20
0
11
10
21
0
0
LINE
8
PARTS
10
10
20
0
11
10
21
10
0
ARC
8
PARTS
10
5
20
10
40
5
50
0
51
180
0
LINE
8
PARTS
10
0
20
10
11
0
21
0
ENDSEC
0
EOF
)DXF";

const char* kBulgeCircleDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LWPOLYLINE
8
BULGE
70
1
10
0
20
0
42
1
10
10
20
0
42
1
ENDSEC
0
EOF
)DXF";

const char* kLegacyPolylineDxf = R"DXF(
0
SECTION
2
ENTITIES
0
POLYLINE
8
LEGACY
70
1
0
VERTEX
10
0
20
0
42
1
0
VERTEX
10
10
20
0
42
1
0
SEQEND
ENDSEC
0
EOF
)DXF";

const char* kMultiplePartsDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LWPOLYLINE
8
PART_A
70
1
10
0
20
0
10
10
20
0
10
10
20
10
10
0
20
10
0
CIRCLE
8
PART_B
10
30
20
5
40
5
ENDSEC
0
EOF
)DXF";


const char* kOpenPolylineJoinedDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LWPOLYLINE
8
JOINED
70
0
10
0
20
0
10
10
20
0
0
LINE
8
JOINED
10
10
20
0
11
10
21
10
0
LINE
8
JOINED
10
10
20
10
11
0
21
10
0
LINE
8
JOINED
10
0
20
10
11
0
21
0
ENDSEC
0
EOF
)DXF";

const char* kDegenerateArcDxf = R"DXF(
0
SECTION
2
ENTITIES
0
ARC
8
BROKEN_ARC
10
0
20
0
40
10
50
45
51
45
ENDSEC
0
EOF
)DXF";

const char* kInvalidDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LINE
8
BROKEN
10
0
20
0
0
ELLIPSE
8
UNSUPPORTED
10
0
20
0
ENDSEC
0
EOF
)DXF";

const char* kRectangleWithHoleDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LINE
8
0
10
0
20
0
11
100
21
0
0
LINE
8
0
10
100
20
0
11
100
21
100
0
LINE
8
0
10
100
20
100
11
0
21
100
0
LINE
8
0
10
0
20
100
11
0
21
0
0
LINE
8
0
10
20
20
20
11
80
21
20
0
LINE
8
0
10
80
20
20
11
80
21
80
0
LINE
8
0
10
80
20
80
11
20
21
80
0
LINE
8
0
10
20
20
80
11
20
21
20
0
ENDSEC
0
EOF
)DXF";

Polygon rectangle(double w, double h) {
    return {{0,0},{w,0},{w,h},{0,h}};
}

void testGeometry() {
    const auto a = rectangle(10, 10);
    const auto b = translate(rectangle(10, 10), 5, 5);
    const auto c = translate(rectangle(10, 10), 20, 0);

    assert(polygonsIntersect(a, b));
    assert(!polygonsIntersect(a, c));
    assert(pointInPolygon({5, 5}, a));
    assert(std::abs(polygonArea(a) - 100.0) < 1e-9);
}

void testDxfHoleRecovery() {
    const auto doc = importDxf(kRectangleWithHoleDxf, 0.1);

    assert(doc.entitiesRead >= 8);
    assert(doc.closedLoopsFound == 2);
    assert(doc.contours.size() == 1);
    assert(doc.contours.front().outer.size() >= 4);
    assert(doc.contours.front().holes.size() == 1);
}

void testLayerSeparation() {
    const auto doc = importDxf(kLayerSeparatedDxf, 0.1);
    assert(doc.closedLoopsFound == 2);
    assert(doc.contours.size() == 2);

    bool sawA = false;
    bool sawB = false;
    for (const auto& contour : doc.contours) {
        if (contour.layer == "A") sawA = true;
        if (contour.layer == "B") sawB = true;
    }
    assert(sawA && sawB);
}


void testLineArcClosure() {
    const auto doc = importDxf(kLineArcClosedDxf, 0.05);
    assert(doc.valid());
    assert(doc.contours.size() == 1);
    assert(doc.closedLoopsFound == 1);
    assert(doc.contours.front().holes.empty());
}

void testBulgePolyline() {
    const auto doc = importDxf(kBulgeCircleDxf, 0.02);
    assert(doc.valid());
    assert(doc.contours.size() == 1);
    const auto& p = doc.contours.front().outer;

    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (const auto& point : p) {
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
    }

    assert(minY < -4.5);
    assert(maxY > 4.5);
    assert(std::abs(polygonArea(p) - 25.0 * 3.14159265358979323846) < 1.0);
}

void testLegacyPolyline() {
    const auto doc = importDxf(kLegacyPolylineDxf, 0.02);
    assert(doc.valid());
    assert(doc.contours.size() == 1);
    assert(doc.supportedEntities == 1);
    assert(!doc.contours.front().layer.empty());
}

void testMultipleParts() {
    const auto doc = importDxf(kMultiplePartsDxf, 0.05);
    assert(doc.valid());
    assert(doc.contours.size() == 2);

    bool sawPartA = false;
    bool sawPartB = false;
    for (const auto& contour : doc.contours) {
        if (contour.layer == "PART_A") sawPartA = true;
        if (contour.layer == "PART_B") sawPartB = true;
    }
    assert(sawPartA && sawPartB);
}


void testOpenPolylineJoining() {
    const auto doc = importDxf(kOpenPolylineJoinedDxf, 0.05);
    assert(doc.valid());
    assert(doc.contours.size() == 1);
    assert(doc.contours.front().outer.size() >= 4);
}

void testDegenerateArc() {
    const auto doc = importDxf(kDegenerateArcDxf, 0.05);
    assert(!doc.valid());
    assert(doc.hasErrors());

    bool foundArcError = false;
    for (const auto& diagnostic : doc.diagnostics) {
        if (diagnostic.stage == "DXF/ARC") {
            foundArcError = true;
            break;
        }
    }
    assert(foundArcError);
}

void testReadableValidationErrors() {
    const auto doc = importDxf(kInvalidDxf, 0.05);
    assert(!doc.valid());
    assert(doc.hasErrors());
    assert(doc.hasWarnings());

    bool hasMissingField = false;
    bool hasUnsupported = false;

    for (const auto& d : doc.diagnostics) {
        if (d.stage == "DXF/VALIDATE") hasMissingField = true;
        if (d.stage == "DXF/ENTITY") hasUnsupported = true;
    }

    assert(hasMissingField);
    assert(hasUnsupported);
}

void testDxfModelPipeline() {
    const auto doc = importDxf(kMultiplePartsDxf, 0.05);
    assert(doc.valid());

    const auto parts = partsFromDxf(doc);
    assert(parts.size() == 2);
    assert(!parts[0].sourceId.empty() || !parts[1].sourceId.empty());

    const auto instances = instancesFromDxf(doc, 2);
    assert(instances.size() == 4);
    assert(instances[0].id != instances[1].id);
}

void testDxfExportRoundTrip() {
    const auto doc = importDxf(kRectangleWithHoleDxf, 0.05);
    assert(doc.valid());

    const auto instances = instancesFromDxf(doc, 1);
    assert(instances.size() == 1);

    Sheet sheet{200, 200, 5};
    Options options;
    options.rotations = {0};
    options.iterations = 4;
    options.gapMm = 2.0;

    const auto result = nest(instances, sheet, options);
    assert(result.unplaced.empty());
    assert(result.sheets.size() == 1);

    DxfExportOptions exportOptions;
    exportOptions.includeSheetOutlines = false;

    const auto exported = exportNestDxf(
        result,
        instances,
        sheet,
        exportOptions
    );

    assert(exported.find("LWPOLYLINE") != std::string::npos);
    assert(exported.find("_HOLE") != std::string::npos);

    const auto roundTrip = importDxf(exported, 0.05);
    assert(roundTrip.valid());
    assert(roundTrip.contours.size() == 1);
    assert(roundTrip.contours.front().holes.size() == 1);
}

void testNfpUnionAndCache() {
    nfp::clearCache();

    Polygon a = rectangle(20, 20);
    Polygon b = rectangle(10, 10);

    const auto first = nfp::noFitPolygons(a, b, 0, 2.0);
    const auto afterFirst = nfp::cacheStats();
    assert(!first.empty());
    assert(afterFirst.misses == 1);
    assert(afterFirst.entries == 1);

    const auto second = nfp::noFitPolygons(
        translate(a, 1000, 1000),
        translate(b, 2000, 3000),
        0,
        2.0
    );
    const auto afterSecond = nfp::cacheStats();

    assert(!second.empty());
    assert(afterSecond.hits == 1);
    assert(afterSecond.misses == 1);
    assert(afterSecond.entries == 1);
}

void testConcaveUnionNfp() {
    nfp::clearCache();

    Polygon fixed{
        {0,0},{40,0},{40,10},{20,10},{20,30},{0,30}
    };
    Polygon moving = rectangle(8, 8);

    const auto unionNfp = nfp::noFitPolygons(fixed, moving, 90, 1.5);
    assert(!unionNfp.empty());

    double totalAbsArea = 0.0;
    for (const auto& polygon : unionNfp) {
        totalAbsArea += std::abs(polygonArea(polygon));
    }
    assert(totalAbsArea > 0.0);
}

double testPointSegmentDistance(Point p, Point a, Point b) {
    const double vx = b.x - a.x;
    const double vy = b.y - a.y;
    const double len2 = vx * vx + vy * vy;
    if (len2 <= 1e-12) return std::hypot(p.x - a.x, p.y - a.y);

    double t = ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2;
    t = std::clamp(t, 0.0, 1.0);

    const Point q{
        a.x + t * vx,
        a.y + t * vy
    };
    return std::hypot(p.x - q.x, p.y - q.y);
}

double testPolygonBoundaryDistance(const Polygon& a, const Polygon& b) {
    double best = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < a.size(); ++i) {
        const Point a0 = a[i];
        const Point a1 = a[(i + 1) % a.size()];

        for (std::size_t j = 0; j < b.size(); ++j) {
            const Point b0 = b[j];
            const Point b1 = b[(j + 1) % b.size()];

            best = std::min(best, testPointSegmentDistance(a0, b0, b1));
            best = std::min(best, testPointSegmentDistance(a1, b0, b1));
            best = std::min(best, testPointSegmentDistance(b0, a0, a1));
            best = std::min(best, testPointSegmentDistance(b1, a0, a1));
        }
    }
    return best;
}

void testContinuousConcaveFeasibilityRegion() {
    Polygon fixed{
        {0,0},{50,0},{50,12},{24,12},{24,34},{0,34}
    };
    Polygon moving = rectangle(8, 8);

    const auto region = nfp::feasibilityRegion(
        fixed,
        moving,
        0,
        -30.0,
        -30.0,
        60.0,
        50.0,
        1.0
    );

    assert(!region.boundary.empty());

    bool foundLongEdge = false;
    bool foundInteriorBoundaryPoint = false;

    for (const auto& segment : region.boundary) {
        const double length =
            std::hypot(
                segment.b.x - segment.a.x,
                segment.b.y - segment.a.y
            );

        if (length < 6.0) continue;
        foundLongEdge = true;

        const Point interior{
            segment.a.x * 0.37 + segment.b.x * 0.63,
            segment.a.y * 0.37 + segment.b.y * 0.63
        };

        const auto sampled =
            nfp::pointsOnFeasibilityBoundary(region, 1000.0);

        for (const auto& point : sampled) {
            const double distance =
                testPointSegmentDistance(
                    point,
                    segment.a,
                    segment.b
                );

            if (distance <= 1e-6 &&
                !(
                    (std::hypot(point.x - segment.a.x, point.y - segment.a.y) < 1e-5) ||
                    (std::hypot(point.x - segment.b.x, point.y - segment.b.y) < 1e-5)
                )) {
                foundInteriorBoundaryPoint = true;
                break;
            }
        }

        (void)interior;
        if (foundInteriorBoundaryPoint) break;
    }

    assert(foundLongEdge);
    assert(foundInteriorBoundaryPoint);
}

void testFeasibilitySamplingBudget() {
    Polygon fixed = rectangle(100, 40);
    Polygon moving = rectangle(20, 10);

    const auto region = nfp::feasibilityRegion(
        fixed,
        moving,
        0,
        -100.0,
        -100.0,
        140.0,
        100.0,
        1.0
    );

    assert(!region.boundary.empty());

    const auto compact = nfp::pointsOnFeasibilityBoundary(
        region,
        0.5,
        32,
        false
    );
    const auto dense = nfp::pointsOnFeasibilityBoundary(
        region,
        0.5,
        256,
        false
    );

    assert(!compact.empty());
    assert(compact.size() <= 32);
    assert(dense.size() <= 256);
    assert(dense.size() >= compact.size());

    bool sawInterior = false;
    for (const auto& segment : region.boundary) {
        const double length = std::hypot(
            segment.b.x - segment.a.x,
            segment.b.y - segment.a.y
        );
        if (length < 15.0) continue;

        for (const auto& point : compact) {
            const double distance = testPointSegmentDistance(
                point,
                segment.a,
                segment.b
            );
            const double da = std::hypot(
                point.x - segment.a.x,
                point.y - segment.a.y
            );
            const double db = std::hypot(
                point.x - segment.b.x,
                point.y - segment.b.y
            );

            if (distance <= 1e-6 && da > 1e-5 && db > 1e-5) {
                sawInterior = true;
                break;
            }
        }

        if (sawInterior) break;
    }

    assert(sawInterior);
}

void testFeasibilityGap() {
    Polygon fixed = rectangle(20, 20);
    Polygon moving = rectangle(10, 10);
    const double gap = 2.0;

    const auto region = nfp::feasibilityRegion(
        fixed,
        moving,
        0,
        -30.0,
        -30.0,
        40.0,
        40.0,
        gap
    );

    assert(!region.boundary.empty());

    bool foundExpectedGap = false;

    for (const auto& segment : region.boundary) {
        const double length =
            std::hypot(
                segment.b.x - segment.a.x,
                segment.b.y - segment.a.y
            );

        if (length < 5.0) continue;

        const Point point{
            (segment.a.x + segment.b.x) * 0.5,
            (segment.a.y + segment.b.y) * 0.5
        };

        const Polygon translatedMoving = translate(
            moving,
            point.x,
            point.y
        );

        const double distance = testPolygonBoundaryDistance(
            translatedMoving,
            fixed
        );

        if (distance >= gap - 1e-5) {
            foundExpectedGap = true;
            break;
        }
    }

    assert(foundExpectedGap);
}

void testCollinearConcaveNfpRegression() {
    Polygon fixed{
        {0,0},
        {15,0},
        {30,0},
        {40,10},
        {25,10},
        {25,30},
        {10,30},
        {0,30}
    };
    Polygon moving = rectangle(8, 6);

    const auto pieces = nfp::convexDecompose(fixed);
    assert(!pieces.empty());

    const auto polygons =
        nfp::noFitPolygons(fixed, moving, 0, 0.0);

    assert(!polygons.empty());
}

void testClearanceCornerSampling() {
    Polygon fixed = rectangle(20, 20);
    Polygon moving = rectangle(10, 10);

    const auto region = nfp::feasibilityRegion(
        fixed,
        moving,
        0,
        -20.0,
        -20.0,
        40.0,
        40.0,
        2.0
    );

    assert(!region.boundary.empty());

    const auto points = nfp::pointsOnFeasibilityBoundary(
        region,
        1.0,
        512,
        false
    );

    assert(!points.empty());

    bool sawDiagonalClearancePoint = false;
    for (const auto& point : points) {
        const double ax = std::abs(point.x);
        const double ay = std::abs(point.y);

        if (ax > 2.0 + 1e-3 &&
            ay > 2.0 + 1e-3) {
            sawDiagonalClearancePoint = true;
            break;
        }
    }

    assert(sawDiagonalClearancePoint);
}

void testNfpMinkowski() {
    const Polygon fixed = rectangle(20, 10);
    const Polygon moving = rectangle(5, 4);

    const auto pieces = nfp::convexDecompose(fixed);
    assert(pieces.size() == 2);

    const auto vertices = nfp::noFitVertices(fixed, moving);
    assert(vertices.size() >= 4);

    bool sawNegativeX = false;
    bool sawPositiveX = false;
    for (const auto& p : vertices) {
        if (p.x < -4.0) sawNegativeX = true;
        if (p.x > 14.0) sawPositiveX = true;
    }
    assert(sawNegativeX && sawPositiveX);
}

void testConcaveNfpCandidates() {
    Polygon concave{
        {0,0},{40,0},{40,10},{20,10},{20,30},{0,30}
    };
    Polygon part = rectangle(8, 8);

    const auto vertices = nfp::noFitVertices(concave, part);
    assert(!vertices.empty());
}

void testPerPartQuantitiesAndUnitIds() {
    const auto doc = importDxf(kMultiplePartsDxf, 0.05);
    assert(doc.valid());

    const std::vector<std::size_t> quantities{2, 3};
    const auto instances = instancesFromDxf(doc, quantities);

    assert(instances.size() == 5);
    assert(instances[0].unitId == "PART_A:unit-1");
    assert(instances[1].unitId == "PART_A:unit-2");
    assert(instances[2].unitId == "PART_B:unit-1");
    assert(instances[4].unitId == "PART_B:unit-3");
}

void testInstanceDiagnostics() {
    std::vector<Instance> instances{
        {"placed-1", Part{"part", rectangle(10, 10), {}}},
        {"missing-1", Part{"part", rectangle(10, 10), {}}}
    };

    Sheet sheet{15, 15, 0};
    Options options;
    options.rotations = {0};
    options.iterations = 1;
    options.gapMm = 0;

    const auto result = nest(instances, sheet, options);
    const auto diagnostics = diagnoseNest(instances, result);

    assert(diagnostics.size() == instances.size());

    bool sawPlaced = false;
    bool sawUnplaced = false;

    for (const auto& diagnostic : diagnostics) {
        if (diagnostic.instanceId == "placed-1") {
            sawPlaced =
                diagnostic.status == InstanceDiagnosticStatus::Placed &&
                diagnostic.stage == "nesting/placed";
        }

        if (diagnostic.instanceId == "missing-1") {
            sawUnplaced =
                diagnostic.status == InstanceDiagnosticStatus::Unplaced &&
                diagnostic.stage == "nesting/no-valid-candidate" &&
                !diagnostic.unitId.empty();
        }
    }

    assert(sawPlaced);
    assert(sawUnplaced);
}

void testSmallPartOptimization() {
    std::vector<Instance> instances{
        {"large-a", Part{"large", rectangle(60, 40), {}}},
        {"large-b", Part{"large", rectangle(60, 40), {}}},
        {"small-a", Part{"small", rectangle(12, 12), {}}},
        {"small-b", Part{"small", rectangle(12, 12), {}}},
        {"small-c", Part{"small", rectangle(12, 12), {}}}
    };

    Sheet sheet{100, 100, 1.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 4;
    options.gapMm = 2.0;
    options.enableSmallPartOptimization = true;
    options.smallPartAreaRatio = 0.10;
    options.smallPartCandidateBudget = 1024;
    options.smallPartBoundarySpacingMm = 1.0;
    options.smallPartRefillPasses = 3;

    const auto result = nest(instances, sheet, options);

    assert(result.unplaced.size() < instances.size());
    assert(result.stats.candidateChecks > 0);

    // The small-part pass may refill residual regions, but it must never
    // violate the configured physical clearance.
    const auto report =
        validateProductionResult(
            instances,
            sheet,
            options,
            result
        );
    assert(report.valid);
}

void testNestingBenchmark() {
    std::vector<Instance> instances;
    for (int i = 0; i < 6; ++i) {
        instances.push_back({
            "bench-" + std::to_string(i),
            Part{"bench", rectangle(20, 20), {}}
        });
    }

    Sheet sheet{100, 100, 0};
    Options options;
    options.rotations = {0, 90, 180, 270};
    options.iterations = 4;
    options.gapMm = 1.0;

    const auto benchmark = benchmarkNest(
        instances,
        sheet,
        options
    );

    assert(benchmark.baseline.milliseconds >= 0.0);
    assert(benchmark.optimized.milliseconds >= 0.0);
    assert(benchmark.baseline.placed <= instances.size());
    assert(benchmark.optimized.placed <= instances.size());
    assert(
        benchmark.baseline.placed +
        benchmark.baseline.skipped ==
        instances.size()
    );
    assert(
        benchmark.optimized.placed +
        benchmark.optimized.skipped ==
        instances.size()
    );
    assert(benchmark.optimized.candidateChecks > 0);
    assert(benchmark.optimized.nfpChecks > 0);

    // The baseline must remain a true greedy reference. If post-processing
    // accidentally leaks back into the baseline, this assertion catches it.
    assert(benchmark.baseline.optimizerPasses == 0);
    assert(benchmark.baseline.refillMoves == 0);
    assert(benchmark.baseline.exchangeAttempts == 0);
    assert(benchmark.baseline.sheetsEliminated == 0);

    // BenchmarkCase must expose the complete optimizer telemetry from nest().
    const auto optimizedDirect = nest(instances, sheet, options);
    assert(
        benchmark.optimized.candidateChecks ==
        optimizedDirect.stats.candidateChecks
    );
    assert(
        benchmark.optimized.collisionChecks ==
        optimizedDirect.stats.collisionChecks
    );
    assert(
        benchmark.optimized.nfpChecks ==
        optimizedDirect.stats.nfpChecks
    );
    assert(
        benchmark.optimized.refillMoves ==
        optimizedDirect.stats.refillMoves
    );
    assert(
        benchmark.optimized.exchangeAttempts ==
        optimizedDirect.stats.exchangeAttempts
    );
    assert(
        benchmark.optimized.sheetsEliminated ==
        optimizedDirect.stats.sheetsEliminated
    );
    assert(
        benchmark.optimized.optimizerPasses ==
        optimizedDirect.stats.optimizerPasses
    );
}

void testSpatialIndexBroadPhase() {
    SpatialIndex index(10.0);
    index.insert(0, Bounds{0.0, 0.0, 5.0, 5.0});
    index.insert(1, Bounds{40.0, 40.0, 45.0, 45.0});
    index.insert(2, Bounds{-35.0, -35.0, 70.0, 70.0});

    const auto nearOrigin = index.query(
        Bounds{4.0, 4.0, 6.0, 6.0},
        0.0
    );

    assert(std::find(
        nearOrigin.begin(),
        nearOrigin.end(),
        0
    ) != nearOrigin.end());

    // Entry 1 is far away, while the large overflow entry 2 must always
    // remain visible to preserve broad-phase completeness.
    assert(std::find(
        nearOrigin.begin(),
        nearOrigin.end(),
        1
    ) == nearOrigin.end());

    assert(std::find(
        nearOrigin.begin(),
        nearOrigin.end(),
        2
    ) != nearOrigin.end());

    const auto padded = index.query(
        Bounds{0.0, 0.0, 5.0, 5.0},
        35.0
    );
    assert(std::find(
        padded.begin(),
        padded.end(),
        1
    ) != padded.end());

    std::vector<Bounds> rebuilt{
        {-100.0, -100.0, -90.0, -90.0},
        {100.0, 100.0, 110.0, 110.0}
    };
    index.rebuild(rebuilt);
    assert(index.size() == rebuilt.size());
    const auto afterRebuild = index.query(
        Bounds{-95.0, -95.0, -85.0, -85.0}
    );
    assert(std::find(
        afterRebuild.begin(),
        afterRebuild.end(),
        0
    ) != afterRebuild.end());
}

void testProductionValidator() {
    std::vector<Instance> instances{
        {"p1", Part{"part-a", rectangle(10, 10), {}}},
        {"p2", Part{"part-b", rectangle(10, 10), {}}},
        {"p3", Part{"part-c", rectangle(10, 10), {}}}
    };

    Sheet sheet{100, 100, 2.0};
    Options options;
    options.rotations = {0};
    options.gapMm = 2.0;

    Result valid;
    valid.sheets = {{
        {"p1", 2.0, 2.0, 0},
        {"p2", 14.0, 2.0, 0},
        {"p3", 26.0, 2.0, 0}
    }};

    const auto validReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            valid
        );

    assert(validReport.valid);
    assert(validReport.collisionCount == 0);
    assert(validReport.gapViolationCount == 0);
    assert(validReport.marginViolationCount == 0);
    assert(validReport.duplicateIdCount == 0);
    assert(validReport.missingIdCount == 0);

    Result collision = valid;
    collision.sheets = {{
        {"p1", 2.0, 2.0, 0},
        {"p2", 5.0, 2.0, 0}
    }};
    collision.unplaced = {"p3"};

    const auto collisionReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            collision
        );

    assert(!collisionReport.valid);
    assert(collisionReport.collisionCount > 0);
    assert(!collisionReport.issues.empty());
    assert(
        collisionReport.issues.front().type ==
        ProductionValidationIssueType::Collision
    );
    assert(
        std::any_of(
            collisionReport.issues.begin(),
            collisionReport.issues.end(),
            [](const ProductionValidationIssue& issue) {
                return issue.type ==
                    ProductionValidationIssueType::MissingId;
            }
        )
    );

    Result gap = valid;
    gap.sheets = {{
        {"p1", 2.0, 2.0, 0},
        {"p2", 13.0, 2.0, 0},
        {"p3", 26.0, 2.0, 0}
    }};

    const auto gapReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            gap
        );

    assert(!gapReport.valid);
    assert(gapReport.gapViolationCount > 0);

    Result margin = valid;
    margin.sheets = {{
        {"p1", 0.0, 2.0, 0},
        {"p2", 14.0, 2.0, 0},
        {"p3", 26.0, 2.0, 0}
    }};

    const auto marginReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            margin
        );

    assert(!marginReport.valid);
    assert(marginReport.marginViolationCount > 0);

    Result duplicate = valid;
    duplicate.sheets = {{
        {"p1", 2.0, 2.0, 0},
        {"p1", 14.0, 2.0, 0},
        {"p3", 26.0, 2.0, 0}
    }};

    const auto duplicateReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            duplicate
        );

    assert(!duplicateReport.valid);
    assert(duplicateReport.duplicateIdCount > 0);

    Result missing = valid;
    missing.sheets = {{
        {"p1", 2.0, 2.0, 0}
    }};

    const auto missingReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            missing
        );

    assert(!missingReport.valid);
    assert(missingReport.missingIdCount == 2);

    Result unexpected = valid;
    unexpected.sheets = {{
        {"p1", 2.0, 2.0, 0},
        {"unknown", 20.0, 2.0, 0},
        {"p3", 26.0, 2.0, 0}
    }};

    const auto unexpectedReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            unexpected
        );

    assert(!unexpectedReport.valid);
    assert(unexpectedReport.unknownIdCount > 0);
}



void testAdaptiveDestroyAndRepair() {
    std::vector<Instance> instances{
        {"adr-1", Part{"adr-a", rectangle(10, 10), {}}},
        {"adr-2", Part{"adr-b", rectangle(10, 10), {}}},
        {"adr-3", Part{"adr-c", rectangle(10, 10), {}}},
        {"adr-4", Part{"adr-d", rectangle(10, 10), {}}}
    };

    Sheet sheet{100, 100, 2.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 2;
    options.gapMm = 2.0;
    options.adaptiveRepairAttempts = 4;
    options.adaptiveRepairMaxNeighbors = 4;
    options.adaptiveRepairRounds = 2;

    Result broken;
    broken.sheets = {{
        {"adr-1", 2.0, 2.0, 0},
        {"adr-2", 5.0, 2.0, 0},
        {"adr-3", 18.0, 2.0, 0},
        {"adr-4", 70.0, 70.0, 0}
    }};

    const double originalFixedX = broken.sheets.front()[2].x;
    const double originalFixedY = broken.sheets.front()[2].y;
    const double originalFarX = broken.sheets.front()[3].x;
    const double originalFarY = broken.sheets.front()[3].y;

    const bool changed =
        adaptiveDestroyAndRepairResult(
            instances,
            sheet,
            options,
            {"adr-1", "adr-2"},
            broken
        );

    assert(changed);

    const auto report =
        validateProductionResult(
            instances,
            sheet,
            options,
            broken
        );

    assert(report.valid);
    assert(broken.sheets.size() == 1);

    bool sawFixed = false;
    bool sawFar = false;

    for (const auto& placement : broken.sheets.front()) {
        if (placement.id == "adr-3") {
            sawFixed = true;
            assert(std::abs(placement.x - originalFixedX) < 1e-7);
            assert(std::abs(placement.y - originalFixedY) < 1e-7);
        }
        if (placement.id == "adr-4") {
            sawFar = true;
            assert(std::abs(placement.x - originalFarX) < 1e-7);
            assert(std::abs(placement.y - originalFarY) < 1e-7);
        }
    }

    assert(sawFixed);
    assert(sawFar);
}


void testAdaptiveRepairLocalityAndDeduplication() {
    std::vector<Instance> instances{
        {"local-1", Part{"local-a", rectangle(10, 10), {}}},
        {"local-2", Part{"local-b", rectangle(10, 10), {}}},
        {"local-3", Part{"local-c", rectangle(10, 10), {}}},
        {"local-4", Part{"local-d", rectangle(10, 10), {}}}
    };

    Sheet sheet{100, 100, 2.0};
    Options options;
    options.rotations = {0};
    options.iterations = 2;
    options.gapMm = 2.0;
    options.adaptiveRepairMaxNeighbors = 4;
    options.adaptiveRepairRounds = 4;

    Result broken;
    broken.sheets = {{
        {"local-1", 2.0, 2.0, 0},
        {"local-2", 5.0, 2.0, 0},
        // local-3 is close by center, but its exact boundary clearance
        // from the conflict is above the configured gap.
        {"local-3", 18.0, 2.0, 0},
        {"local-4", 70.0, 70.0, 0}
    }};

    const double stableX = broken.sheets.front()[2].x;
    const double stableY = broken.sheets.front()[2].y;
    const double farX = broken.sheets.front()[3].x;
    const double farY = broken.sheets.front()[3].y;

    const bool changed =
        adaptiveDestroyAndRepairResult(
            instances,
            sheet,
            options,
            {"local-1", "local-2"},
            broken
        );

    assert(changed);

    const auto report =
        validateProductionResult(
            instances,
            sheet,
            options,
            broken
        );
    assert(report.valid);

    for (const auto& placement : broken.sheets.front()) {
        if (placement.id == "local-3") {
            assert(std::abs(placement.x - stableX) < 1e-7);
            assert(std::abs(placement.y - stableY) < 1e-7);
        }
        if (placement.id == "local-4") {
            assert(std::abs(placement.x - farX) < 1e-7);
            assert(std::abs(placement.y - farY) < 1e-7);
        }
    }

    // Repeating the same local conflict must not create duplicate repair
    // rounds in the production-repair history.
    Result repeatBroken;
    repeatBroken.sheets = {{
        {"local-1", 2.0, 2.0, 0},
        {"local-2", 5.0, 2.0, 0},
        {"local-3", 18.0, 2.0, 0}
    }};
    repeatBroken.unplaced = {"local-4"};

    Options repairOptions = options;
    repairOptions.autoRepairAttempts = 1;
    repairOptions.autoRepairTimeBudgetMs = 3000;
    repairOptions.adaptiveRepairRounds = 4;

    ProductionValidationReport repairReport;
    const bool repaired =
        repairProductionResult(
            instances,
            sheet,
            repairOptions,
            repeatBroken,
            &repairReport
        );

    assert(repaired);
    assert(repairReport.valid);

    std::vector<std::string> signatures;
    for (const auto& round : repairReport.adaptiveHistory) {
        auto ids = round.conflictIds;
        std::sort(ids.begin(), ids.end());
        std::string signature;
        for (const auto& id : ids) {
            signature += id;
            signature.push_back('|');
        }
        signatures.push_back(std::move(signature));
    }

    std::sort(signatures.begin(), signatures.end());
    assert(std::adjacent_find(signatures.begin(), signatures.end()) ==
           signatures.end());
}


void testAdaptiveRepairConflictGraph() {
    std::vector<Instance> instances{
        {"graph-a", Part{"graph-a-part", rectangle(10, 10), {}}},
        {"graph-b", Part{"graph-b-part", rectangle(10, 10), {}}},
        {"graph-c", Part{"graph-c-part", rectangle(10, 10), {}}},
        {"graph-d", Part{"graph-d-part", rectangle(10, 10), {}}}
    };

    Sheet sheet{100, 100, 2.0};
    Options options;
    options.rotations = {0};
    options.iterations = 2;
    options.gapMm = 2.0;
    options.adaptiveRepairMaxNeighbors = 8;
    options.adaptiveRepairRounds = 3;
    options.autoRepairAttempts = 1;
    options.autoRepairTimeBudgetMs = 5000;

    Result broken;
    broken.sheets = {{
        {"graph-a", 2.0, 2.0, 0},
        {"graph-b", 8.0, 2.0, 0},
        {"graph-c", 14.0, 2.0, 0},
        {"graph-d", 30.0, 2.0, 0}
    }};

    const auto initial = validateProductionResult(instances, sheet, options, broken);
    assert(!initial.valid);
    assert(initial.collisionCount >= 2);

    ProductionValidationReport report;
    const bool repaired = repairProductionResult(instances, sheet, options, broken, &report);
    assert(repaired);
    assert(report.valid);
    assert(report.adaptiveRepairRounds >= 1);
    assert(report.adaptiveRepairGroupSize >= 3);

    bool sawDStationary = false;
    for (const auto& change : report.adaptiveChanges) {
        if (change.before.id == "graph-d") {
            sawDStationary = change.stationary;
        }
    }
    if (!report.adaptiveChanges.empty()) {
        assert(sawDStationary);
    }
}


void testAutomaticProductionRepair() {
    std::vector<Instance> instances{
        {"repair-1", Part{"repair-a", rectangle(10, 10), {}}},
        {"repair-2", Part{"repair-b", rectangle(10, 10), {}}},
        {"repair-3", Part{"repair-c", rectangle(10, 10), {}}}
    };

    Sheet sheet{100, 100, 2.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 2;
    options.gapMm = 2.0;
    options.autoRepairAttempts = 4;
    options.autoRepairTimeBudgetMs = 3000;

    Result broken;
    broken.sheets = {{
        {"repair-1", 2.0, 2.0, 0},
        {"repair-2", 5.0, 2.0, 0}
    }};
    broken.unplaced = {"repair-3"};

    ProductionValidationReport report;
    const bool repaired =
        repairProductionResult(
            instances,
            sheet,
            options,
            broken,
            &report
        );

    assert(repaired);
    assert(report.valid);
    assert(report.repaired);
    assert(report.repairAttempts > 0);
    assert(broken.productionValidated);
    assert(broken.productionValid);
    assert(broken.productionIssueCount == 0);

    const auto finalReport =
        validateProductionResult(
            instances,
            sheet,
            options,
            broken
        );
    assert(finalReport.valid);
    assert(broken.unplaced.empty());
}


void testCandidateCollectorAndGlobalOptimizer() {
    NestingCandidateCollector collector(2);

    Result threeSheets;
    threeSheets.sheets.resize(3);
    threeSheets.unplaced = {"x"};
    threeSheets.utilization = 0.2;

    Result twoSheets;
    twoSheets.sheets.resize(2);
    twoSheets.unplaced = {};
    twoSheets.utilization = 0.4;

    Result fourSheets;
    fourSheets.sheets.resize(4);
    fourSheets.unplaced = {};
    fourSheets.utilization = 0.9;

    collector.add(threeSheets);
    collector.add(twoSheets);
    collector.add(fourSheets);

    const auto candidates = collector.snapshot();
    assert(candidates.size() == 2);
    assert(candidates.front().sheets.size() == 2);

    std::vector<Instance> instances;
    for (int i = 0; i < 3; ++i) {
        instances.push_back({
            "global-" + std::to_string(i),
            Part{"global", rectangle(50, 50), {}}
        });
    }

    Sheet sheet{100, 100, 0};
    Options options;
    options.rotations = {0};
    options.iterations = 1;
    options.gapMm = 2.0;
    options.enableOptimizer = false;

    auto result = nest(instances, sheet, options);
    const auto beforeSheets = result.sheets.size();
    const auto beforeUnplaced = result.unplaced.size();

    const bool changed =
        optimizeNestingResult(
            instances,
            sheet,
            options,
            result
        );

    (void)changed;
    assert(result.unplaced.size() <= beforeUnplaced);
    assert(result.sheets.size() <= beforeSheets);
    assert(result.stats.optimizerPasses > 0);
}

void testParallelNestingController() {
    std::vector<Instance> instances;
    for (int i = 0; i < 8; ++i) {
        instances.push_back({
            "parallel-" + std::to_string(i),
            Part{"parallel", rectangle(20, 20), {}}
        });
    }

    Sheet sheet{100, 100, 0};
    Options options;
    options.rotations = {0, 90, 180, 270};
    options.gapMm = 1.0;

    ParallelNestingController controller;
    ParallelNestingOptions parallel;
    parallel.workers = 2;
    parallel.iterations = 6;
    parallel.timeBudgetMs = 10000;

    std::atomic<std::size_t> progressEvents{0};
    std::atomic<bool> completed{false};
    std::atomic<bool> validated{false};

    parallel.onProgress =
        [&](const NestingProgress& progress) {
            ++progressEvents;
            if (progress.phase ==
                NestingProgressPhase::ProductionValidation) {
                validated.store(true);
            }
            if (progress.phase ==
                NestingProgressPhase::Completed) {
                completed.store(true);
            }
        };

    parallel.onValidation =
        [&](const ProductionValidationReport& report) {
            validated.store(report.valid);
        };

    const auto result = controller.run(
        instances,
        sheet,
        options,
        parallel
    );

    assert(progressEvents.load() > 0);
    assert(validated.load());
    assert(completed.load());
    assert(result.unplaced.empty());
    assert(result.sheets.size() <= 2);
    assert(controller.cancelRequested() == false);
}

void testParallelNestingCancellation() {
    std::vector<Instance> instances;
    for (int i = 0; i < 10; ++i) {
        instances.push_back({
            "cancel-" + std::to_string(i),
            Part{"cancel", rectangle(20, 20), {}}
        });
    }

    Sheet sheet{100, 100, 0};
    Options options;
    options.rotations = {0, 90, 180, 270};
    options.gapMm = 1.0;

    ParallelNestingController controller;
    ParallelNestingOptions parallel;
    parallel.workers = 2;
    parallel.iterations = 32;
    parallel.timeBudgetMs = 10000;

    std::atomic<std::size_t> completed{0};

    parallel.onProgress =
        [&](const NestingProgress& progress) {
            if (progress.phase ==
                NestingProgressPhase::IterationFinished) {
                const auto count =
                    completed.fetch_add(
                        1,
                        std::memory_order_relaxed
                    ) + 1;

                if (count == 1) {
                    controller.requestCancel();
                }
            }
        };

    const auto result = controller.run(
        instances,
        sheet,
        options,
        parallel
    );

    (void)result;
    assert(controller.cancelRequested());
    assert(completed.load(std::memory_order_relaxed) < parallel.iterations);
}

void testMinimumSheets() {
    std::vector<Instance> parts;
    for (int i = 0; i < 3; ++i) {
        parts.push_back({ "p" + std::to_string(i), Part{"rect", rectangle(50, 50), {}} });
    }

    Sheet sheet{100, 100, 0};
    Options options;
    options.rotations = {0};
    options.iterations = 16;
    options.gapMm = 2.0;

    const auto result = nest(parts, sheet, options);

    assert(result.unplaced.empty());
    assert(result.sheets.size() == 2);
}

void testInterlockIntoHole() {
    Part host;
    host.id = "host";
    host.outer = rectangle(100, 100);
    host.holes.push_back({{20,20},{80,20},{80,80},{20,80}});

    Part insert;
    insert.id = "insert";
    insert.outer = rectangle(50, 50);

    std::vector<Instance> parts{
        {"host-1", host},
        {"insert-1", insert}
    };

    Sheet sheet{110, 110, 0};
    Options options;
    options.rotations = {0};
    options.iterations = 8;
    options.gapMm = 2.0;

    const auto result = nest(parts, sheet, options);

    assert(result.unplaced.empty());
    assert(result.sheets.size() == 1);

    bool foundInsert = false;
    for (const auto& placement : result.sheets.front()) {
        if (placement.id != "insert-1") continue;
        foundInsert = true;
        assert(placement.x >= 20.0 - 1e-6);
        assert(placement.y >= 20.0 - 1e-6);
        assert(placement.x + 50.0 <= 80.0 + 1e-6);
        assert(placement.y + 50.0 <= 80.0 + 1e-6);
    }
    assert(foundInsert);
}

} // namespace

int main() {
    testGeometry();
    testDxfHoleRecovery();
    testLayerSeparation();
    testLineArcClosure();
    testBulgePolyline();
    testLegacyPolyline();
    testMultipleParts();
    testOpenPolylineJoining();
    testDegenerateArc();
    testDxfModelPipeline();
    testPerPartQuantitiesAndUnitIds();
    testDxfExportRoundTrip();
    testCollinearConcaveNfpRegression();
    testClearanceCornerSampling();
    testNfpMinkowski();
    testContinuousConcaveFeasibilityRegion();
    testFeasibilitySamplingBudget();
    testFeasibilityGap();
    testNfpUnionAndCache();
    testConcaveUnionNfp();
    testConcaveNfpCandidates();
    testReadableValidationErrors();
    testMinimumSheets();
    testProductionValidator();
    testSpatialIndexBroadPhase();
    testAdaptiveDestroyAndRepair();
    testAdaptiveRepairLocalityAndDeduplication();
    testAdaptiveRepairConflictGraph();
    testAutomaticProductionRepair();
}
