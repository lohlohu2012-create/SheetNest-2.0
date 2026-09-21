#include "sheetnest/dxf.hpp"
#include "sheetnest/dxf_export.hpp"
#include "sheetnest/dxf_model.hpp"
#include "sheetnest/geometry.hpp"
#include "sheetnest/nesting.hpp"
#include "sheetnest/nfp.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

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
    testDxfExportRoundTrip();
    testNfpMinkowski();
    testConcaveNfpCandidates();
    testReadableValidationErrors();
    testMinimumSheets();
    testInterlockIntoHole();
    std::cout << "SheetNest core smoke tests passed\n";
    return 0;
}
