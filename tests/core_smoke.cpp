#include "sheetnest/dxf.hpp"
#include "sheetnest/geometry.hpp"
#include "sheetnest/nesting.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace sheetnest;

namespace {

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
}

} // namespace

int main() {
    testGeometry();
    testDxfHoleRecovery();
    testMinimumSheets();
    testInterlockIntoHole();
    std::cout << "SheetNest core smoke tests passed\n";
    return 0;
}
