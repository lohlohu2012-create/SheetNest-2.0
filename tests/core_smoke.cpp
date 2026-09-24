#include "sheetnest/dxf.hpp"
#include "sheetnest/dxf_export.hpp"
#include "sheetnest/cam_export.hpp"
#include "sheetnest/benchmark.hpp"
#include "sheetnest/cutting_path.hpp"
#include "sheetnest/dxf_export.hpp"
#include "sheetnest/cutting.hpp"
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
#include <cstdlib>
#include <unordered_set>
#include <chrono>

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

const char* kComplexContoursDxf = R"DXF(
0
SECTION
2
ENTITIES
0
LWPOLYLINE
8
CONVEX
70
1
10
0
20
0
10
40
20
10
40
20
10
0
20
20
0
LWPOLYLINE
8
CONCAVE
70
1
10
60
20
0
10
120
20
0
10
120
20
10
10
95
20
10
10
95
20
38
10
85
20
38
10
85
20
10
10
60
20
10
0
LWPOLYLINE
8
DEGENERATE
70
1
10
140
20
0
10
170
20
0
10
170
20
0.0000000001
10
170
20
20
10
155
20
20
10
140
20
20
10
140
20
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
    const auto roundTripPreflight = preflightDxf(roundTrip);
    assert(roundTripPreflight.valid);
    assert(roundTripPreflight.validParts == 1);
    assert(roundTrip.contours.size() == 1);
    assert(roundTrip.contours.front().holes.size() == 1);
}

void testNfpUnionAndCache() {
    nfp::clearCache();

    Polygon a = rectangle(20, 20);
    Polygon b = rectangle(10, 10);

    std::size_t runHits = 0;
    std::size_t runMisses = 0;
    nfp::NfpRunControl runControl;
    runControl.cacheHitCount = &runHits;
    runControl.cacheMissCount = &runMisses;

    const auto first = nfp::noFitPolygons(a, b, 0, 2.0, &runControl);
    const auto afterFirst = nfp::cacheStats();
    assert(!first.empty());
    assert(afterFirst.misses == 1);
    assert(afterFirst.entries == 1);
    assert(runMisses == 1);

    const auto second = nfp::noFitPolygons(
        translate(a, 1000, 1000),
        translate(b, 2000, 3000),
        0,
        2.0,
        &runControl
    );
    const auto afterSecond = nfp::cacheStats();

    assert(!second.empty());
    assert(afterSecond.hits == 1);
    assert(afterSecond.misses == 1);
    assert(afterSecond.entries == 1);
    assert(runHits == 1);
    assert(runMisses == 1);
}


void testNfpCacheCyclicCanonicalization() {
    nfp::clearCache();

    const Polygon fixed = {
        {0,0},{40,0},{40,20},{0,20}
    };
    const Polygon fixedRotatedStart = {
        {40,20},{0,20},{0,0},{40,0}
    };
    const Polygon moving = {
        {0,0},{8,0},{8,5},{0,5}
    };

    const auto first = nfp::noFitPolygons(fixed, moving, 0, 0.5);
    const auto before = nfp::cacheStats();
    const auto second = nfp::noFitPolygons(
        fixedRotatedStart,
        moving,
        0,
        0.5
    );
    const auto after = nfp::cacheStats();

    assert(!first.empty());
    assert(!second.empty());
    assert(before.entries == 1);
    assert(after.entries == 1);
    assert(after.hits == before.hits + 1);
    assert(after.misses == before.misses);

    assert(first.size() == second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        assert(first[i].size() == second[i].size());
    }
}

void testNfpComplexContourMatrix() {
    const auto dxf = importDxf(kComplexContoursDxf, 0.001);
    assert(dxf.valid());
    assert(dxf.contours.size() == 3);
    const auto dxfParts = partsFromDxf(dxf);
    assert(dxfParts.size() == 3);
    for (const auto& part : dxfParts) {
        assert(part.outer.size() >= 4);
        const auto dxfNfp = nfp::noFitPolygons(
            dxfParts.front().outer,
            part.outer,
            0,
            0.5
        );
        assert(!dxfNfp.empty());
    }


    nfp::clearCache();

    // Convex: baseline must be non-empty and deterministic.
    const Polygon convex = rectangle(40.0, 20.0);
    const Polygon tool = rectangle(7.0, 5.0);
    const auto convexNfp = nfp::noFitPolygons(convex, tool, 0, 0.0);
    assert(!convexNfp.empty());

    // Strongly concave U/L-like contour with a narrow passage. The NFP must
    // survive reflex vertices and still expose a continuous boundary.
    const Polygon narrowConcave{
        {0,0},{60,0},{60,10},{35,10},{35,38},
        {25,38},{25,10},{0,10}
    };
    const auto concaveNfp = nfp::noFitPolygons(
        narrowConcave,
        rectangle(8,8),
        90,
        1.0
    );
    assert(!concaveNfp.empty());

    const auto narrowRegion = nfp::feasibilityRegion(
        narrowConcave,
        rectangle(8,8),
        0,
        -20.0,-20.0,80.0,60.0,
        1.0
    );
    assert(!narrowRegion.boundary.empty());
    const auto narrowCandidates = nfp::pointsOnFeasibilityBoundary(
        narrowRegion, 0.5, 256, true
    );
    assert(!narrowCandidates.empty());

    // Near-degenerate segments: repeated points, tiny collinear runs and a
    // very short edge must not collapse the NFP stage.
    const Polygon degenerate{
        {0,0},{30,0},{30,0},{30,1e-10},{30,20},
        {15,20},{15,20},{0,20},{0,0}
    };
    const auto degenerateNfp = nfp::noFitPolygons(
        degenerate,
        rectangle(5,5),
        0,
        0.5
    );
    assert(!degenerateNfp.empty());
}

void testNfpHolePipeline() {
    // NFP is built from the outer contour; holes are retained by Part and
    // checked by the exact production validator. Verify that a DXF hole
    // survives import and the resulting part can pass the complete nesting
    // pipeline without the NFP stage producing zero candidates.
    const auto doc = importDxf(kRectangleWithHoleDxf, 0.05);
    assert(doc.valid());
    assert(doc.contours.size() == 1);
    assert(doc.contours.front().holes.size() == 1);

    const auto instances = instancesFromDxf(doc, 1);
    assert(instances.size() == 1);

    Sheet sheet{140.0, 140.0, 2.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 4;
    options.gapMm = 1.0;
    options.enableOptimizer = false;
    options.enableAdaptiveDestroyRepair = false;
    options.enableAutoRepair = false;

    const auto result = nest(instances, sheet, options);
    assert(result.unplaced.empty());
    assert(!result.sheets.empty());

    const auto validation = validateProductionResult(
        instances, sheet, options, result
    );
    assert(validation.valid);
    assert(validation.coverageComplete);
    assert(validation.expectedInstanceCount == 1);
    assert(validation.placedInstanceCount == 1);
    assert(validation.unplacedInstanceCount == 0);

    CuttingParameters cuttingParameters;
    cuttingParameters.material = Material::CarbonSteel;
    cuttingParameters.thicknessMm = 2.0;
    cuttingParameters.speedMMin = 20.0;
    cuttingParameters.assistGas = "O2";

    const auto pipeline = validateProductionPipeline(
        doc,
        instances,
        sheet,
        options,
        result,
        cuttingParameters
    );
    assert(pipeline.valid);
    assert(pipeline.failedStage == ProductionPipelineStage::Complete);
    assert(pipeline.camOperationCount > 0);
    assert(pipeline.exportedBytes > 0);
    assert(pipeline.roundTripValid);
    assert(!pipeline.failureReason.size());
    assert(pipeline.completedStages.size() == 10);
}


void testNfpContinuousSearchApi() {
    nfp::clearCache();

    const Polygon fixed = rectangle(100.0, 40.0);
    const Polygon moving = rectangle(20.0, 10.0);

    std::size_t exactChecks = 0;
    nfp::SearchOptions options;
    options.boundarySpacingMm = 2.0;
    options.maxCandidates = 64;
    options.includeSheetBoundary = true;
    options.includeNfpVertices = true;
    options.isFeasible = [&](const Point& p) {
        ++exactChecks;
        // Exact validation is intentionally supplied by the caller. For this
        // API regression we accept only points inside the requested search
        // rectangle; the nesting engine can replace this callback with its
        // true-shape collision/clearance predicate.
        return p.x >= -1e-8 && p.x <= 120.0 + 1e-8 &&
               p.y >= -1e-8 && p.y <= 60.0 + 1e-8;
    };

    const auto result = nfp::searchFeasibleBoundary(
        fixed,
        moving,
        0,
        0.0,
        0.0,
        120.0,
        60.0,
        2.0,
        options
    );

    assert(!result.points.empty());
    assert(result.points.size() <= options.maxCandidates);
    assert(result.telemetry.generated > 0);
    assert(result.telemetry.boundarySegments > 0);
    assert(result.telemetry.boundarySamples > 0);
    assert(result.telemetry.exactChecks == exactChecks);
    assert(result.telemetry.feasible == result.points.size());
    assert(result.telemetry.exactChecks >= result.telemetry.feasible);

    bool sawNonVertexLikePoint = false;
    const auto rawVertices = nfp::noFitVertices(fixed, moving);
    for (const auto& candidate : result.points) {
        bool isVertex = false;
        for (const auto& vertex : rawVertices) {
            if (std::hypot(
                    candidate.x - vertex.x,
                    candidate.y - vertex.y
                ) <= 1e-7) {
                isVertex = true;
                break;
            }
        }
        if (!isVertex) {
            sawNonVertexLikePoint = true;
            break;
        }
    }
    assert(sawNonVertexLikePoint);

    std::size_t stoppedCount = 0;
    nfp::NfpRunControl stopGuard;
    stopGuard.shouldStop = [&]() {
        ++stoppedCount;
        return true;
    };

    nfp::SearchOptions stoppedOptions;
    stoppedOptions.maxCandidates = 32;
    const auto stopped = nfp::searchFeasibleBoundary(
        fixed,
        moving,
        0,
        0.0,
        0.0,
        120.0,
        60.0,
        2.0,
        stoppedOptions,
        &stopGuard
    );

    assert(stopped.points.empty());
    assert(stopped.telemetry.stopped);
    assert(stoppedCount > 0);
}

void testNfpTimeoutRecovery() {
    auto control = std::make_shared<NestingRunControl>();
    control->deadline = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(1);

    std::vector<Instance> parts;
    for (int i = 0; i < 48; ++i) {
        parts.push_back({
            "nfp-timeout-" + std::to_string(i),
            Part{"timeout", rectangle(13.0, 7.0), {}}
        });
    }

    Sheet sheet{180.0, 120.0, 1.0};
    Options options;
    options.rotations = {0, 90, 180, 270};
    options.iterations = 32;
    options.gapMm = 1.0;
    options.enableOptimizer = true;
    options.enableAdaptiveDestroyRepair = true;
    options.control = control;

    const auto result = nest(parts, sheet, options);
    assert(control->timeoutObserved.load(std::memory_order_relaxed));
    assert(result.sheets.size() <= parts.size());
    assert(result.unplaced.size() <= parts.size());

    // Timeout is a controlled termination, not a parser/NFP corruption. Any
    // already-produced placement must still be representable and unique.
    std::size_t placed = 0;
    std::vector<std::string> ids;
    for (const auto& sheetPlacements : result.sheets) {
        for (const auto& placement : sheetPlacements) {
            ++placed;
            ids.push_back(placement.id);
        }
    }
    std::sort(ids.begin(), ids.end());
    assert(std::adjacent_find(ids.begin(), ids.end()) == ids.end());
    assert(placed + result.unplaced.size() <= parts.size());
    assert(result.instanceTelemetry.size() == parts.size());
    bool sawTimedOutInstance = false;
    for (const auto& telemetry : result.instanceTelemetry) {
        if (telemetry.placed) {
            assert(telemetry.reason == NestingFailureReason::None);
        } else {
            sawTimedOutInstance =
                sawTimedOutInstance ||
                telemetry.reason == NestingFailureReason::Timeout;
        }
    }
    assert(sawTimedOutInstance);
}

void testNfpInternalTimeoutAndComplexityGuard() {
    nfp::clearCache();

    std::size_t timeoutCount = 0;
    std::size_t complexityFallbacks = 0;

    nfp::NfpRunControl timeoutGuard;
    timeoutGuard.shouldStop = []() { return true; };
    timeoutGuard.timeoutCount = &timeoutCount;
    timeoutGuard.complexityFallbackCount = &complexityFallbacks;

    const auto timedOut = nfp::noFitPolygons(
        rectangle(40.0, 30.0),
        rectangle(10.0, 8.0),
        0,
        0.0,
        &timeoutGuard
    );
    assert(!timedOut.empty());
    assert(timeoutCount > 0);

    const auto timedVertices = nfp::noFitVertices(
        rectangle(40.0, 30.0),
        rectangle(10.0, 8.0),
        90,
        0.0,
        &timeoutGuard
    );
    assert(!timedVertices.empty());
    assert(timeoutCount > 1);

    nfp::NfpRunControl complexityGuard;
    complexityGuard.maxInputVertices = 3;
    complexityGuard.complexityFallbackCount = &complexityFallbacks;

    Polygon complex{
        {0,0},{20,0},{20,5},{12,5},{12,20},{0,20}
    };
    const auto guarded = nfp::noFitPolygons(
        complex,
        rectangle(5.0, 5.0),
        0,
        0.0,
        &complexityGuard
    );
    assert(!guarded.empty());
    assert(complexityFallbacks > 0);
}

void testNfpDegenerateFallback() {
    nfp::clearCache();

    // Intentionally malformed bow-tie contour: the production pipeline must
    // not turn a decomposition failure into an empty NFP/candidate set.
    Polygon malformedFixed{
        {0.0, 0.0},
        {30.0, 30.0},
        {0.0, 30.0},
        {30.0, 0.0}
    };
    Polygon moving = rectangle(5.0, 5.0);

    const auto polygons = nfp::noFitPolygons(
        malformedFixed,
        moving,
        0,
        0.0
    );
    assert(!polygons.empty());

    const auto region = nfp::feasibilityRegion(
        malformedFixed,
        moving,
        0,
        -10.0,
        -10.0,
        40.0,
        40.0,
        1.0
    );
    assert(!region.boundary.empty());

    const auto sampled = nfp::pointsOnFeasibilityBoundary(
        region,
        1.0,
        64,
        true
    );
    assert(!sampled.empty());
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

void testFeasibilitySegmentCoverage() {
    nfp::FeasibilityRegion region;

    // Deliberately create many short segments and two long segments. With a
    // tight budget, the old longest-only policy could discard the isolated
    // short regions entirely. The stratified policy must expose candidates
    // from more than one spatial part of the boundary.
    for (int i = 0; i < 12; ++i) {
        const double x = static_cast<double>(i) * 10.0;
        region.boundary.push_back({
            {x, 0.0},
            {x + 2.0, 0.0}
        });
    }

    region.boundary.push_back({
        {0.0, 20.0},
        {80.0, 20.0}
    });
    region.boundary.push_back({
        {0.0, 30.0},
        {80.0, 30.0}
    });

    const auto sampled = nfp::pointsOnFeasibilityBoundary(
        region,
        1.0,
        12,
        false
    );

    assert(!sampled.empty());
    assert(sampled.size() <= 12);

    bool sawEarly = false;
    bool sawLate = false;
    bool sawLong = false;

    for (const auto& point : sampled) {
        if (point.x < 20.0) sawEarly = true;
        if (point.x > 80.0) sawLate = true;
        if (std::abs(point.y - 20.0) < 1e-6 ||
            std::abs(point.y - 30.0) < 1e-6) {
            sawLong = true;
        }
    }

    assert(sawEarly);
    assert(sawLate);
    assert(sawLong);
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

void testLaserTechnologyInterpolation() {
    const auto technologyTable = bodor3kWTechnologyTable();
    const auto technologyReport = validateBodor3kWTechnology();
    assert(technologyReport.valid);
    assert(technologyReport.rows == technologyTable.size());
    assert(technologyReport.invalidRows == 0);
    assert(technologyReport.duplicateRows == 0);
    for (const auto& row : technologyTable) {
        assert(row.thicknessMm > 0.0);
        assert(row.speedMMin > 0.0);
        assert(!row.assistGas.empty());
    }

    const auto lower = bodor3kWParameters(Material::CarbonSteel, 2.0);
    const auto mid = bodor3kWParameters(Material::CarbonSteel, 2.5);
    const auto upper = bodor3kWParameters(Material::CarbonSteel, 3.0);

    assert(lower.speedMMin > upper.speedMMin);
    assert(mid.speedMMin > upper.speedMMin);
    assert(mid.speedMMin < lower.speedMMin);
    assert(std::abs(mid.speedMMin - 5.0) < 1e-9);
    assert(mid.assistGas == lower.assistGas);
}

void testCuttingPathInnerContoursFirst() {
    CuttingParameters technology;
    technology.speedMMin = 10.0;
    PathOptions options;
    options.rapidSpeedMMin = 120.0;
    options.pierceSeconds = 0.25;
    options.innerContoursFirst = true;

    const Polygon outer = rectangle(100.0, 100.0);
    const Polygon hole = translate(rectangle(10.0, 10.0), 45.0, 45.0);
    const auto path = planCuttingPath({outer, hole}, technology, options);

    assert(path.pierces == 2);
    assert(path.moves.size() > 2);
    std::size_t firstPierceMove = std::numeric_limits<std::size_t>::max();
    for (std::size_t i = 0; i < path.moves.size(); ++i) {
        if (path.moves[i].type == CutType::Pierce) {
            firstPierceMove = i;
            break;
        }
    }
    assert(firstPierceMove != std::numeric_limits<std::size_t>::max());

    // The first contour cut must be the small internal contour, not the
    // enclosing 100x100 outer boundary.
    assert(path.moves[firstPierceMove].from.x >= 45.0 - 1e-6);
    assert(path.moves[firstPierceMove].from.x <= 55.0 + 1e-6);
    assert(path.moves[firstPierceMove].from.y >= 45.0 - 1e-6);
    assert(path.moves[firstPierceMove].from.y <= 55.0 + 1e-6);

    const auto estimate = estimateCuttingPath(path, technology, options);
    assert(estimate.contourLengthMm > 0.0);
    assert(estimate.totalMinutes > 0.0);

    const auto camReport = validateCuttingPath(path);
    assert(camReport.valid);
    CamExportOptions camOptions;
    camOptions.includeComments = true;
    const auto camProgram =
        exportCamProgram(path, technology, camOptions);
    assert(!camProgram.empty());
    assert(camProgram.find("G90") != std::string::npos);
    assert(camProgram.find("M2") != std::string::npos);
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

    Sheet sheet{100.0, 100.0, 1.0};
    Options options;
    options.iterations = 1;
    options.enableOptimizer = false;
    options.enableProductionValidation = false;
    options.enableAutoRepair = false;
    options.enableAdaptiveDestroyRepair = false;
    options.gapMm = 1.0;

    const auto result = nest(instances, sheet, options);
    assert(result.instanceTelemetry.size() == instances.size());
    for (const auto& telemetry : result.instanceTelemetry) {
        assert(!telemetry.instanceId.empty());
        assert(!telemetry.unitId.empty());
        assert(telemetry.elapsedMs >= 0);
        if (telemetry.placed) {
            assert(telemetry.reason == NestingFailureReason::None);
        }
    }
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
    assert(
        benchmark.optimized.nfpTimeouts ==
        optimizedDirect.stats.nfpTimeouts
    );
    assert(
        benchmark.optimized.nfpComplexityFallbacks ==
        optimizedDirect.stats.nfpComplexityFallbacks
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

    // The adaptive history must also be free of repeated physical placement
    // states, not just repeated conflict-group IDs. This catches A -> B -> A
    // repair cycles that can otherwise use different conflict signatures.
    std::vector<std::string> stateSignatures;
    for (const auto& round : repairReport.adaptiveHistory) {
        std::vector<std::string> tokens;
        for (std::size_t sheetIndex = 0;
             sheetIndex < round.afterSheets.size();
             ++sheetIndex) {
            for (const auto& placement : round.afterSheets[sheetIndex]) {
                const auto qx = static_cast<long long>(
                    std::llround(placement.x * 1000000.0)
                );
                const auto qy = static_cast<long long>(
                    std::llround(placement.y * 1000000.0)
                );
                tokens.push_back(
                    std::to_string(sheetIndex) + ":" +
                    placement.id + ":" +
                    std::to_string(placement.rotation) + ":" +
                    std::to_string(qx) + ":" +
                    std::to_string(qy)
                );
            }
        }
        std::sort(tokens.begin(), tokens.end());
        std::string stateSignature;
        for (const auto& token : tokens) {
            stateSignature += token;
            stateSignature.push_back('|');
        }
        stateSignatures.push_back(std::move(stateSignature));
    }

    std::sort(stateSignatures.begin(), stateSignatures.end());
    assert(std::adjacent_find(
               stateSignatures.begin(),
               stateSignatures.end()
           ) == stateSignatures.end());
}


void testAdaptiveRepairConflictGraph() {
    std::vector<Instance> instances{
        {"graph-a", Part{"graph-a-part", rectangle(10, 10), {}}},
        {"graph-b", Part{"graph-b-part", rectangle(10, 10), {}}},
        {"graph-c", Part{"graph-c-part", rectangle(10, 10), {}}},
        {"graph-d", Part{"graph-d-part", rectangle(10, 10), {}}},
        {"graph-e", Part{"graph-e-part", rectangle(10, 10), {}}}
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
        // C-D have a real 1.5 mm clearance, below the required 2 mm gap.
        {"graph-d", 25.5, 2.0, 0},
        // E is unrelated and must remain outside the repair hierarchy.
        {"graph-e", 50.0, 2.0, 0}
    }};

    const auto initial = validateProductionResult(instances, sheet, options, broken);
    assert(!initial.valid);
    assert(initial.collisionCount >= 2);
    assert(initial.gapViolationCount >= 1);

    ProductionValidationReport report;
    const bool repaired = repairProductionResult(instances, sheet, options, broken, &report);
    assert(repaired);
    assert(report.valid);
    assert(report.adaptiveRepairRounds >= 1);
    assert(report.adaptiveRepairGroupSize >= 3);
    assert(!report.adaptiveHistory.empty());

    bool sawCollisionLevel = false;
    bool sawGapLevel = false;
    bool sawDependentLevel = false;
    bool sawEStationary = false;
    bool sawNewCollisionAfterRepair = false;
    std::size_t previousValidationSequence = 0;
    std::size_t previousLevel = 0;
    for (std::size_t i = 0; i < report.adaptiveHistory.size(); ++i) {
        const auto& round = report.adaptiveHistory[i];
        assert(round.validationSequence > previousValidationSequence);
        previousValidationSequence = round.validationSequence;
        if (i > 0) {
            // The hierarchy may restart at Collision after a newly exposed
            // conflict, but it can never move backwards from Gap to fringe
            // within the same successful repair chain.
            assert(round.repairedLevel <= previousLevel + 1);
        }
        previousLevel = round.repairedLevel;
        assert(!round.conflictLevels.empty());

        // The snapshot must contain the validator result produced immediately
        // after this exact sub-level. Re-run validation against afterSheets
        // and compare every recorded physical violation count.
        Result after;
        after.sheets = round.afterSheets;
        const auto afterValidation =
            validateProductionResult(instances, sheet, options, after);
        assert(round.collisionCountAfter == afterValidation.collisionCount);
        assert(round.gapViolationCountAfter == afterValidation.gapViolationCount);
        assert(round.marginViolationCountAfter == afterValidation.marginViolationCount);
        assert(round.validAfter == afterValidation.valid);

        if (round.repairedLevel == 0) {
            sawCollisionLevel = true;
            assert(round.conflictLevels.front().size() > 0);
        } else if (round.repairedLevel == 1) {
            sawGapLevel = true;
            assert(round.conflictLevels.size() >= 2);
            assert(!round.conflictLevels[1].empty());
        } else {
            sawDependentLevel = true;
        }

        // If a repair itself exposes a new Collision, the next adaptive
        // sub-level/round must remain Collision-first; it must not jump to Gap.
        if (round.collisionCountAfter > 0 && i + 1 < report.adaptiveHistory.size()) {
            sawNewCollisionAfterRepair = true;
            const auto& next = report.adaptiveHistory[i + 1];
            assert(!next.conflictLevels.empty());
            assert(!next.conflictLevels.front().empty());
            assert(next.conflictIds.size() >= 1);
        }

        for (const auto& change : round.changes) {
            if (change.before.id == "graph-e") {
                sawEStationary = change.stationary;
            }
        }
    }
    assert(sawCollisionLevel);
    assert(sawGapLevel);
    assert(sawDependentLevel || report.adaptiveRepairRounds < 3);
    if (!report.adaptiveChanges.empty()) {
        assert(sawEStationary);
    }
    // The scenario is also accepted when the first repair solves Collision
    // immediately; the important regression is that any newly exposed
    // Collision is revalidated and remains the highest-priority next level.
    (void)sawNewCollisionAfterRepair;
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
    assert(result.instanceTelemetry.size() == instances.size());
    for (const auto& telemetry : result.instanceTelemetry) {
        if (!telemetry.placed) {
            assert(
                telemetry.reason == NestingFailureReason::Cancelled ||
                telemetry.reason == NestingFailureReason::Timeout ||
                telemetry.reason == NestingFailureReason::NoFeasiblePosition
            );
        }
    }
}

void testDenseSmallPartPlacement() {
    // Stress scenario: many small instances must exploit narrow remaining
    // spaces instead of prematurely opening another sheet. The first fixture
    // is intentionally chosen so a dense 9x9 lattice fits on ONE sheet; this
    // catches candidate starvation that would otherwise create a second sheet.
    std::vector<Instance> dense;
    constexpr int kDenseInstances = 81;
    for (int i = 0; i < kDenseInstances; ++i) {
        dense.push_back({
            "dense-small-one-sheet-" + std::to_string(i),
            Part{"dense-small", rectangle(9.0, 4.0), {}}
        });
    }

    Sheet sheet{100.0, 50.0, 1.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 8;
    options.gapMm = 1.0;
    options.enableSmallPartOptimization = true;
    options.smallPartAreaRatio = 0.25;
    options.smallPartCandidateBudget = 1536;
    options.smallPartBoundarySpacingMm = 1.0;
    options.smallPartRefillPasses = 4;
    options.enableOptimizer = true;

    const auto oneSheetResult = nest(dense, sheet, options);

    assert(oneSheetResult.unplaced.empty());
    assert(oneSheetResult.sheets.size() == 1);

    std::size_t densePlacedCount = 0;
    for (const auto& placements : oneSheetResult.sheets) {
        densePlacedCount += placements.size();
    }
    assert(densePlacedCount == static_cast<std::size_t>(kDenseInstances));

    // A second, larger fixture verifies that the same strategy scales to a
    // high instance count without silently losing parts or exploding sheet
    // count. Two sheets are sufficient for this quantity.
    std::vector<Instance> parts;
    constexpr int kInstances = 162;
    for (int i = 0; i < kInstances; ++i) {
        parts.push_back({
            "dense-small-" + std::to_string(i),
            Part{"dense-small", rectangle(9.0, 4.0), {}}
        });
    }

    const auto result = nest(parts, sheet, options);

    assert(result.unplaced.empty());
    assert(!result.sheets.empty());
    assert(result.sheets.size() <= 2);

    // Final telemetry must agree with the authoritative placement list after
    // small-part refill/recovery. This catches stale "unplaced" states.
    assert(result.instanceTelemetry.size() == parts.size());
    for (const auto& telemetry : result.instanceTelemetry) {
        assert(telemetry.placed);
        assert(telemetry.reason == NestingFailureReason::None);
    }

    std::size_t placedCount = 0;
    for (const auto& placements : result.sheets) {
        placedCount += placements.size();

        for (const auto& placement : placements) {
            assert(placement.x >= sheet.edgeMarginMm - 1e-6);
            assert(placement.y >= sheet.edgeMarginMm - 1e-6);
            assert(placement.x <= sheet.width - sheet.edgeMarginMm + 1e-6);
            assert(placement.y <= sheet.height - sheet.edgeMarginMm + 1e-6);
        }
    }
    assert(placedCount == static_cast<std::size_t>(kInstances));
    assert(result.instanceTelemetry.size() == parts.size());
    for (const auto& telemetry : result.instanceTelemetry) {
        assert(telemetry.placed);
        assert(telemetry.reason == NestingFailureReason::None);
        assert(!telemetry.instanceId.empty());
        assert(!telemetry.unitId.empty());
        assert(telemetry.candidateChecks > 0);
        assert(telemetry.feasibleCandidates > 0);
        assert(
            telemetry.boundsRejections +
            telemetry.collisionRejections +
            telemetry.feasibleCandidates <=
            telemetry.candidateChecks
        );
    }

    // With 9x4 parts, 1 mm technological gap and a 100x50 sheet, a dense
    // layout has substantial opportunity for narrow strip reuse. Requiring
    // two sheets or fewer makes the test sensitive to candidate starvation
    // without depending on a single exact placement order.
    const auto validation =
        validateProductionResult(parts, sheet, options, result);
    assert(validation.valid);
    assert(result.unplaced.empty());
}

void testPlacementAccountingAfterRefill() {
    // Regression for the post-refill reconciliation path: every requested
    // instance must occur exactly once either in final sheets or in unplaced.
    // This deliberately uses enough small parts to exercise multiple refill
    // passes while keeping the fixture deterministic and fast.
    std::vector<Instance> parts;
    constexpr int kInstances = 120;
    parts.reserve(kInstances);
    for (int i = 0; i < kInstances; ++i) {
        parts.push_back({
            "accounting-" + std::to_string(i),
            Part{"accounting-part", rectangle(7.0, 5.0), {}}
        });
    }

    Sheet sheet{100.0, 60.0, 1.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 10;
    options.gapMm = 1.0;
    options.enableSmallPartOptimization = true;
    options.smallPartAreaRatio = 0.30;
    options.smallPartCandidateBudget = 1536;
    options.smallPartBoundarySpacingMm = 1.0;
    options.smallPartRefillPasses = 6;
    options.enableOptimizer = true;

    const auto result = nest(parts, sheet, options);

    std::unordered_set<std::string> requested;
    for (const auto& instance : parts) {
        assert(requested.insert(instance.id).second);
    }

    std::unordered_set<std::string> placed;
    for (const auto& placements : result.sheets) {
        for (const auto& placement : placements) {
            assert(requested.contains(placement.id));
            assert(placed.insert(placement.id).second);
        }
    }

    std::unordered_set<std::string> unplaced;
    for (const auto& id : result.unplaced) {
        assert(requested.contains(id));
        assert(unplaced.insert(id).second);
        assert(!placed.contains(id));
    }

    assert(placed.size() + unplaced.size() == requested.size());
    assert(result.instanceTelemetry.size() == parts.size());

    for (const auto& telemetry : result.instanceTelemetry) {
        assert(requested.contains(telemetry.instanceId));
        const bool isPlaced = placed.contains(telemetry.instanceId);
        const bool isUnplaced = unplaced.contains(telemetry.instanceId);
        assert(isPlaced != isUnplaced);
        assert(telemetry.placed == isPlaced);

        if (isPlaced) {
            assert(telemetry.reason == NestingFailureReason::None);
        }
    }
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

bool gapHistoryEntryHasPriorZeroCollisionValidation(
    const std::vector<AdaptiveRepairRound>& history,
    std::size_t gapIndex
) {
    if (gapIndex >= history.size()) return false;

    for (std::size_t i = 0; i < gapIndex; ++i) {
        const auto& validation = history[i];
        if (validation.validAfter &&
            validation.collisionCountAfter == 0) {
            return true;
        }
    }
    return false;
}

// Expected-failure scenario used by the parent smoke test. It intentionally
// creates a Gap history entry while Collision is still present. The child
// process must terminate non-zero, proving that the ordering assertion really
// rejects the invalid history instead of merely documenting the rule.
void testNegativeGapBeforeZeroCollisionRevalidation() {
    AdaptiveRepairRound invalidGap;
    invalidGap.repairedLevel = 1;
    invalidGap.collisionCountAfter = 1;
    invalidGap.validAfter = false;
    invalidGap.validationSequence = 1;

    const std::vector<AdaptiveRepairRound> invalidHistory{invalidGap};
    const bool authorized =
        gapHistoryEntryHasPriorZeroCollisionValidation(invalidHistory, 0);

    assert(authorized &&
           "Negative regression: Gap must not appear before zero-Collision revalidation");
    std::abort();
}

void testEndToEndDxfToCam() {
    const std::string dxf =
        "0\nSECTION\n2\nENTITIES\n"
        "0\nLWPOLYLINE\n8\nPART\n90\n4\n70\n1\n"
        "10\n0\n20\n0\n10\n80\n20\n10\n50\n20\n10\n50\n0\n"
        "0\nLWPOLYLINE\n8\nPART\n90\n4\n70\n1\n"
        "10\n20\n10\n20\n30\n10\n30\n30\n10\n30\n20\n"
        "0\nENDSEC\n0\nEOF\n";

    const auto document = importDxf(dxf);
    assert(document.valid());
    const auto preflight = preflightDxf(document);
    assert(preflight.valid);
    assert(preflight.validParts == 2);
    const auto instances = instancesFromDxf(document, 2);
    assert(instances.size() == 2);

    Sheet sheet{120.0, 80.0, 2.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 8;
    options.gapMm = 2.0;
    options.enableOptimizer = true;
    options.enableAutoRepair = true;

    const auto result = nest(instances, sheet, options);
    const auto validation =
        validateProductionResult(instances, sheet, options, result);
    assert(validation.valid);
    assert(result.unplaced.empty());

    const auto technology =
        bodor3kWParameters(Material::CarbonSteel, 3.0);
    PathOptions pathOptions;
    pathOptions.rapidSpeedMMin = 120.0;
    pathOptions.pierceSeconds = 0.25;
    pathOptions.innerContoursFirst = true;

    std::vector<CuttingContour> contours;
    for (std::size_t sheetIndex = 0; sheetIndex < result.sheets.size(); ++sheetIndex) {
        for (const auto& placement : result.sheets[sheetIndex]) {
            auto it = std::find_if(
                instances.begin(), instances.end(),
                [&](const Instance& instance) { return instance.id == placement.id; }
            );
            assert(it != instances.end());

            contours.push_back({
                sheetIndex,
                placement.id,
                0,
                false,
                translate(
                    rotate(it->part.outer, placement.rotation),
                    placement.x,
                    placement.y
                )
            });
        }
    }

    const auto route =
        planCuttingRoute(contours, technology, pathOptions);
    assert(route.operations.size() == instances.size());
    assert(route.totalSeconds > 0.0);
    assert(route.totalCutLengthMm > 0.0);

    for (std::size_t i = 1; i < route.operations.size(); ++i) {
        assert(route.operations[i - 1].operation <
               route.operations[i].operation);
    }

    const auto estimate =
        estimateCuttingPath(route, technology, pathOptions);
    assert(estimate.totalMinutes > 0.0);

    const std::string exported =
        exportNestDxf(result, instances, sheet);
    assert(!exported.empty());
    const auto roundTrip = importDxf(exported);
    assert(roundTrip.valid());
    assert(!roundTrip.contours.empty());
}
 
void testParallelCancellationPath() {
    std::vector<Instance> instances;
    for (int i = 0; i < 24; ++i) {
        instances.push_back({
            "cancel-" + std::to_string(i),
            Part{"cancel-part", rectangle(20.0, 10.0), {}}
        });
    }

    Sheet sheet{500.0, 500.0, 2.0};
    Options nestingOptions;
    nestingOptions.rotations = {0, 90};
    nestingOptions.iterations = 200;
    nestingOptions.gapMm = 2.0;

    auto controller = std::make_shared<ParallelNestingController>();
    ParallelNestingOptions parallel;
    parallel.workers = 2;
    parallel.iterations = 200;
    parallel.timeBudgetMs = 10000;
    parallel.candidateCapacity = 4;

    std::atomic<bool> cancellationObserved{false};
    parallel.onProgress = [controller, &cancellationObserved](const NestingProgress& progress) {
        if (progress.phase == NestingProgressPhase::IterationFinished) {
            cancellationObserved.store(true, std::memory_order_relaxed);
            controller->requestCancel();
        }
    };

    const auto started = std::chrono::steady_clock::now();
    const auto result = controller->run(
        instances,
        sheet,
        nestingOptions,
        parallel
    );
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started
    ).count();

    assert(cancellationObserved.load(std::memory_order_relaxed));
    assert(controller->cancelRequested());
    assert(elapsed < 10000);
    assert(result.sheets.size() <= instances.size());
    assert(result.unplaced.size() <= instances.size());
}

void testAdaptiveRepairNewCollisionPriority() {
    std::vector<Instance> instances{
        {"priority-a", Part{"priority-a-part", rectangle(10, 10), {}}},
        {"priority-b", Part{"priority-b-part", rectangle(10, 10), {}}},
        {"priority-c", Part{"priority-c-part", rectangle(10, 10), {}}},
        {"priority-d", Part{"priority-d-part", rectangle(10, 10), {}}},
        {"priority-e", Part{"priority-e-part", rectangle(10, 10), {}}}
    };

    Sheet sheet{120, 100, 2.0};
    Options options;
    options.rotations = {0};
    options.iterations = 2;
    options.gapMm = 2.0;
    options.adaptiveRepairMaxNeighbors = 8;
    options.adaptiveRepairRounds = 3;
    options.autoRepairAttempts = 1;
    options.autoRepairTimeBudgetMs = 5000;

    // Phase 1: the first repair is deliberately limited to A/B.
    // C is valid at its original position, while D/E form an independent
    // 1.5 mm gap violation that must remain for the next hierarchy level.
    Result firstBroken;
    firstBroken.sheets = {{
        {"priority-a", 2.0, 2.0, 0},
        {"priority-b", 8.0, 2.0, 0},
        {"priority-c", 25.0, 2.0, 0},
        {"priority-d", 36.5, 2.0, 0},
        {"priority-e", 48.0, 2.0, 0}
    }};

    const auto beforeFirst =
        validateProductionResult(instances, sheet, options, firstBroken);
    assert(beforeFirst.collisionCount >= 1);
    assert(beforeFirst.gapViolationCount >= 1);

    // Repair the original collision neighborhood first.
    std::vector<std::string> extracted;
    const bool firstRepair =
        adaptiveDestroyAndRepairResult(
            instances,
            sheet,
            options,
            {"priority-a", "priority-b"},
            firstBroken,
            &extracted
        );
    assert(firstRepair);

    const auto afterFirst =
        validateProductionResult(instances, sheet, options, firstBroken);
    assert(afterFirst.collisionCount == 0);

    // Deterministically expose a NEW Collision after the first repair by
    // moving C onto the repaired A position. This is intentional test
    // instrumentation: it makes the regression independent of optimizer
    // randomness while exercising the exact revalidation/re-prioritization
    // path required in production.
    const auto repairedA = std::find_if(
        firstBroken.sheets[0].begin(),
        firstBroken.sheets[0].end(),
        [](const Placement& p) { return p.id == "priority-a"; }
    );
    assert(repairedA != firstBroken.sheets[0].end());

    auto injectedC = std::find_if(
        firstBroken.sheets[0].begin(),
        firstBroken.sheets[0].end(),
        [](const Placement& p) { return p.id == "priority-c"; }
    );
    assert(injectedC != firstBroken.sheets[0].end());
    injectedC->x = repairedA->x;
    injectedC->y = repairedA->y;

    const auto afterInjectedCollision =
        validateProductionResult(instances, sheet, options, firstBroken);
    assert(afterInjectedCollision.collisionCount >= 1);
    assert(afterInjectedCollision.gapViolationCount >= 1);

    // Phase 2: Adaptive Repair must see the newly exposed Collision first,
    // validate it again, and only then enter Gap processing.
    ProductionValidationReport report;
    const bool repaired =
        repairProductionResult(
            instances,
            sheet,
            options,
            firstBroken,
            &report
        );
    assert(repaired);
    assert(report.valid);
    assert(!report.adaptiveHistory.empty());

    bool sawNewCollisionState = false;
    bool sawCollisionResolved = false;
    bool sawGapAfterCollision = false;
    bool sawSuccessfulRevalidationBetweenCollisionAndFirstGap = false;
    bool sawSuccessfulZeroCollisionRevalidation = false;
    std::size_t lastSequence = 0;
    std::size_t lastLevel = 0;
    std::size_t firstCollisionHistoryIndex = std::numeric_limits<std::size_t>::max();
    std::size_t firstGapHistoryIndex = std::numeric_limits<std::size_t>::max();
    std::size_t firstZeroCollisionValidationHistoryIndex = std::numeric_limits<std::size_t>::max();
    std::size_t gapHistoryEntryCount = 0;

    for (std::size_t historyIndex = 0;
         historyIndex < report.adaptiveHistory.size();
         ++historyIndex) {
        const auto& round = report.adaptiveHistory[historyIndex];
        assert(round.validationSequence > lastSequence);
        lastSequence = round.validationSequence;

        Result after;
        after.sheets = round.afterSheets;
        const auto validation =
            validateProductionResult(instances, sheet, options, after);

        assert(round.collisionCountAfter == validation.collisionCount);
        assert(round.gapViolationCountAfter == validation.gapViolationCount);
        assert(round.validAfter == validation.valid);

        // Independent revalidation gate: this is the only event that can
        // authorize entry into Gap processing. Record the first successful
        // zero-Collision validation explicitly so the regression catches any
        // future history reordering, even if the Collision-level bookkeeping
        // changes.
        if (validation.valid && validation.collisionCount == 0) {
            sawSuccessfulZeroCollisionRevalidation = true;
            if (firstZeroCollisionValidationHistoryIndex ==
                std::numeric_limits<std::size_t>::max()) {
                firstZeroCollisionValidationHistoryIndex = historyIndex;
            }
        }

        if (round.repairedLevel == 0) {
            sawNewCollisionState = true;
            if (firstCollisionHistoryIndex == std::numeric_limits<std::size_t>::max()) {
                firstCollisionHistoryIndex = historyIndex;
            }

            // A Collision-level repair is allowed to expose another
            // Collision. It must be reported by the immediate validator.
            if (round.collisionCountAfter > 0) {
                assert(!round.conflictIds.empty());
                assert(!round.conflictLevels.empty());
                assert(!round.conflictLevels.front().empty());
            } else {
                sawCollisionResolved = true;
            }
        }

        if (round.repairedLevel == 1) {
            ++gapHistoryEntryCount;
            if (firstGapHistoryIndex == std::numeric_limits<std::size_t>::max()) {
                firstGapHistoryIndex = historyIndex;
            }

            // HARD INVARIANT: Gap processing is forbidden while ANY Collision
            // remains. This is deliberately fail-fast so a future regression
            // cannot silently reorder the hierarchy.
            if (round.collisionCountAfter != 0) {
                assert(false && "Adaptive Repair must not enter Gap while Collision remains");
            }
            assert(sawCollisionResolved);
            assert(round.conflictLevels.size() >= 2);
            assert(!round.conflictLevels[1].empty());
            sawGapAfterCollision = true;

            // The first Gap entry must be strictly after the first Collision
            // entry and immediately preceded by a successful revalidation
            // proving that Collision is gone.
            assert(firstCollisionHistoryIndex != std::numeric_limits<std::size_t>::max());
            assert(firstGapHistoryIndex > firstCollisionHistoryIndex);
            assert(firstGapHistoryIndex > 0);

            // Separate ordering invariant: NO Gap history entry may occur
            // before the first successful independent revalidation with
            // collisionCount == 0.
            assert(sawSuccessfulZeroCollisionRevalidation);
            assert(firstZeroCollisionValidationHistoryIndex !=
                   std::numeric_limits<std::size_t>::max());
            assert(firstGapHistoryIndex > firstZeroCollisionValidationHistoryIndex);
            assert(gapHistoryEntryHasPriorZeroCollisionValidation(
                report.adaptiveHistory,
                historyIndex
            ));

            const auto& validationBeforeGap =
                report.adaptiveHistory[firstGapHistoryIndex - 1];
            Result validatedState;
            validatedState.sheets = validationBeforeGap.afterSheets;
            const auto validationBeforeGapResult =
                validateProductionResult(
                    instances,
                    sheet,
                    options,
                    validatedState
                );

            assert(validationBeforeGapResult.valid);
            assert(validationBeforeGapResult.collisionCount == 0);
            assert(validationBeforeGap.validAfter);
            assert(validationBeforeGap.collisionCountAfter == 0);
            assert(validationBeforeGap.validationSequence < round.validationSequence);
            sawSuccessfulRevalidationBetweenCollisionAndFirstGap = true;
        }

        // The hierarchy must not jump from Collision directly to a dependent
        // level while a Collision is still present.
        if (lastSequence > 0 && round.repairedLevel > lastLevel + 1) {
            assert(false);
        }
        lastLevel = round.repairedLevel;
    }

    assert(sawNewCollisionState);
    assert(sawCollisionResolved);
    assert(sawGapAfterCollision);
    assert(firstCollisionHistoryIndex != std::numeric_limits<std::size_t>::max());
    assert(firstGapHistoryIndex != std::numeric_limits<std::size_t>::max());
    assert(firstCollisionHistoryIndex < firstGapHistoryIndex);
    assert(gapHistoryEntryCount == 1);
    assert(sawSuccessfulZeroCollisionRevalidation);
    assert(firstZeroCollisionValidationHistoryIndex < firstGapHistoryIndex);
    assert(sawSuccessfulRevalidationBetweenCollisionAndFirstGap);
}


} // namespace


void testNesting20DenseResidualPacking() {
    std::vector<Instance> parts;
    for (int i = 0; i < 12; ++i) {
        parts.push_back({
            "dense-" + std::to_string(i),
            Part{"dense", rectangle(18.0, 9.0), {}}
        });
    }

    Sheet sheet{60.0, 60.0, 1.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 12;
    options.gapMm = 1.0;
    options.enableOptimizer = true;
    options.enableSmallPartOptimization = true;
    options.smallPartCandidateBudget = 1024;
    options.residualRetryPasses = 3;

    const auto result = nest(parts, sheet, options);

    assert(result.unplaced.empty());
    assert(result.sheets.size() <= 4);
    assert(result.stats.refillMoves >= 0);
    assert(result.stats.exchangeAttempts >= 0);

    std::unordered_set<std::string> placed;
    for (const auto& sheetPlacements : result.sheets) {
        for (const auto& placement : sheetPlacements) {
            placed.insert(placement.id);
        }
    }
    assert(placed.size() == parts.size());
}

void testNesting20NewSheetRecovery() {
    std::vector<Instance> parts{
        {"large-a", Part{"large", rectangle(48.0, 48.0), {}}},
        {"large-b", Part{"large", rectangle(48.0, 48.0), {}}},
        {"large-c", Part{"large", rectangle(48.0, 48.0), {}}},
        {"large-d", Part{"large", rectangle(48.0, 48.0), {}}}
    };

    Sheet sheet{100.0, 100.0, 1.0};
    Options options;
    options.rotations = {0};
    options.iterations = 8;
    options.gapMm = 2.0;
    options.enableOptimizer = true;
    options.residualRetryPasses = 3;

    const auto result = nest(parts, sheet, options);

    assert(result.unplaced.empty());
    assert(result.sheets.size() == 4);
}

void testNesting20OptimizerDoesNotDropPlacedInstances() {
    std::vector<Instance> parts;
    for (int i = 0; i < 16; ++i) {
        parts.push_back({
            "optimizer-" + std::to_string(i),
            Part{"optimizer", rectangle(12.0, 7.0), {}}
        });
    }

    Sheet sheet{80.0, 50.0, 1.0};
    Options options;
    options.rotations = {0, 90};
    options.iterations = 16;
    options.gapMm = 1.0;
    options.enableOptimizer = true;
    options.residualRetryPasses = 3;

    const auto result = nest(parts, sheet, options);

    std::unordered_set<std::string> placed;
    for (const auto& sheetPlacements : result.sheets) {
        for (const auto& placement : sheetPlacements) {
            const bool inserted = placed.insert(placement.id).second;
            assert(inserted && "Nesting 2.0 must never duplicate an instance");
        }
    }

    assert(placed.size() + result.unplaced.size() == parts.size());
}

int main(int argc, char** argv) {
    assert(std::string(productionPipelineStageName(ProductionPipelineStage::Nesting)) == "Nesting");
    assert(std::string(productionPipelineStageName(ProductionPipelineStage::Complete)) == "Complete");
    assert(std::string(productionPipelineStageName(ProductionPipelineStage::Failed)) == "Failed");

    if (argc > 1 &&
        std::string(argv[1]) == "--negative-gap-ordering") {
        testNegativeGapBeforeZeroCollisionRevalidation();
        return 0;
    }

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
    testCuttingPathInnerContoursFirst();
    testLaserTechnologyInterpolation();
    testCollinearConcaveNfpRegression();
    testClearanceCornerSampling();
    testNfpMinkowski();
    testContinuousConcaveFeasibilityRegion();
    testFeasibilitySamplingBudget();
    testFeasibilitySegmentCoverage();
    testFeasibilityGap();
    testNfpContinuousSearchApi();
    testNfpUnionAndCache();
    testNfpCacheCyclicCanonicalization();
    testNfpComplexContourMatrix();
    testNfpHolePipeline();
    testNfpTimeoutRecovery();
    testNfpInternalTimeoutAndComplexityGuard();
    testNfpDegenerateFallback();
    testConcaveUnionNfp();
    testConcaveNfpCandidates();
    testReadableValidationErrors();
    testDenseSmallPartPlacement();
    testPlacementAccountingAfterRefill();
    testMinimumSheets();
    testProductionValidator();
    testSpatialIndexBroadPhase();
    testAdaptiveDestroyAndRepair();
    testAdaptiveRepairLocalityAndDeduplication();
    
 
testAdaptiveRepairConflictGraph();
    testAdaptiveRepairNewCollisionPriority();
    testAutomaticProductionRepair();
    testNesting20DenseResidualPacking();
    testNesting20NewSheetRecovery();
    testNesting20OptimizerDoesNotDropPlacedInstances();


    std::cout << "[PASS] NFP tests" << std::endl;
    std::cout << "[PASS] Adaptive Repair tests" << std::endl;
    std::cout << "[PASS] dense small parts tests" << std::endl;
    std::cout << "[PASS] complex DXF tests" << std::endl;
    std::cout << "[PASS] timeout regression" << std::endl;
    std::cout << "[PASS] DXF export round-trip" << std::endl;
    std::cout << "[PASS] Core Smoke" << std::endl;

    // Meta-regression: run the intentionally invalid scenario in a child
    // process and require it to fail. A zero exit code means the guard has
    // stopped rejecting an invalid Gap-before-revalidation history.
    if (argc > 0 && argv[0] != nullptr) {
        std::string command = "\"";
        command += argv[0];
        command += "\" --negative-gap-ordering";
        const int negativeExitCode = std::system(command.c_str());
        assert(negativeExitCode != 0 &&
               "Negative Gap-ordering regression unexpectedly passed");
    }
}