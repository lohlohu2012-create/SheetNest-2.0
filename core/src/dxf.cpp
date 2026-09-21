#include "sheetnest/dxf.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace sheetnest {

namespace {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEps = 1e-7;

struct GroupCode {
    int code{};
    std::string value;
};

struct Entity {
    std::string type;
    std::vector<GroupCode> groups;
};

struct Segment {
    Polygon points;
    Point start{};
    Point end{};
    std::string source;
    bool used{false};
};

struct Loop {
    Polygon polygon;
    std::string source;
    double signedArea{};
};

double toNumber(const std::string& value) {
    try {
        return std::stod(value);
    } catch (...) {
        return 0.0;
    }
}

int toInt(const std::string& value) {
    try {
        return std::stoi(value);
    } catch (...) {
        return 0;
    }
}

std::vector<GroupCode> readGroups(const std::string& text) {
    std::istringstream stream(text);
    std::string codeLine;
    std::string valueLine;
    std::vector<GroupCode> result;

    while (std::getline(stream, codeLine) && std::getline(stream, valueLine)) {
        if (!codeLine.empty() && codeLine.back() == '\r') {
            codeLine.pop_back();
        }
        if (!valueLine.empty() && valueLine.back() == '\r') {
            valueLine.pop_back();
        }

        try {
            result.push_back({std::stoi(codeLine), valueLine});
        } catch (...) {
            // Ignore malformed pair; diagnostics are emitted by the caller.
        }
    }

    return result;
}

std::vector<Entity> readEntities(const std::string& text, std::size_t& entitiesRead) {
    const auto groups = readGroups(text);
    std::vector<Entity> entities;

    bool inEntities = false;

    for (std::size_t i = 0; i < groups.size();) {
        if (groups[i].code != 0) {
            ++i;
            continue;
        }

        if (groups[i].value == "SECTION") {
            std::size_t j = i + 1;
            std::string sectionName;
            while (j < groups.size() && groups[j].code != 0) {
                if (groups[j].code == 2) {
                    sectionName = groups[j].value;
                    break;
                }
                ++j;
            }
            inEntities = (sectionName == "ENTITIES");
            i = j;
            continue;
        }

        if (groups[i].value == "ENDSEC") {
            inEntities = false;
            ++i;
            continue;
        }

        if (!inEntities) {
            ++i;
            continue;
        }

        Entity entity;
        entity.type = groups[i].value;
        ++entitiesRead;
        ++i;

        while (i < groups.size() && groups[i].code != 0) {
            entity.groups.push_back(groups[i]);
            ++i;
        }

        entities.push_back(std::move(entity));
    }

    return entities;
}

double signedArea(const Polygon& polygon) {
    if (polygon.size() < 3) {
        return 0.0;
    }

    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = polygon[i];
        const auto& b = polygon[(i + 1) % polygon.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return 0.5 * area;
}

bool samePoint(const Point& a, const Point& b, double tolerance = 1e-6) {
    return std::hypot(a.x - b.x, a.y - b.y) <= tolerance;
}

void appendUnique(Polygon& polygon, const Point& point) {
    if (polygon.empty() || !samePoint(polygon.back(), point)) {
        polygon.push_back(point);
    }
}

Point groupPoint(const Entity& entity, int xCode, int yCode, Point fallback = {}) {
    Point result = fallback;
    bool haveX = false;
    bool haveY = false;

    for (const auto& group : entity.groups) {
        if (group.code == xCode) {
            result.x = toNumber(group.value);
            haveX = true;
        } else if (group.code == yCode) {
            result.y = toNumber(group.value);
            haveY = true;
        }
    }

    if (!haveX || !haveY) {
        return fallback;
    }
    return result;
}

double groupValue(const Entity& entity, int code, double fallback = 0.0) {
    for (const auto& group : entity.groups) {
        if (group.code == code) {
            return toNumber(group.value);
        }
    }
    return fallback;
}

int groupInt(const Entity& entity, int code, int fallback = 0) {
    for (const auto& group : entity.groups) {
        if (group.code == code) {
            return toInt(group.value);
        }
    }
    return fallback;
}

void appendArc(
    Polygon& output,
    Point center,
    double radius,
    double startAngle,
    double sweepAngle,
    double tolerance
) {
    if (radius <= kEps || std::abs(sweepAngle) <= kEps) {
        return;
    }

    const double clampedTolerance =
        std::max(0.01, std::min(std::abs(tolerance), radius * 0.5));

    double step = kTwoPi;
    if (clampedTolerance < radius) {
        const double c = std::clamp(1.0 - clampedTolerance / radius, -1.0, 1.0);
        step = 2.0 * std::acos(c);
    }

    step = std::max(step, kPi / 1800.0);
    const int segments = std::max(
        2,
        static_cast<int>(std::ceil(std::abs(sweepAngle) / step))
    );

    for (int i = 0; i <= segments; ++i) {
        const double t =
            startAngle + sweepAngle * static_cast<double>(i) / segments;
        appendUnique(output, {
            center.x + radius * std::cos(t),
            center.y + radius * std::sin(t)
        });
    }
}

Polygon makeCircle(Point center, double radius, double tolerance) {
    Polygon result;
    appendArc(result, center, radius, 0.0, kTwoPi, tolerance);
    if (!result.empty() && samePoint(result.front(), result.back())) {
        result.pop_back();
    }
    return result;
}

Polygon makeBulgeSegment(
    Point start,
    Point end,
    double bulge,
    double tolerance
) {
    Polygon result;
    appendUnique(result, start);

    if (std::abs(bulge) <= kEps || samePoint(start, end)) {
        appendUnique(result, end);
        return result;
    }

    const double chord = std::hypot(end.x - start.x, end.y - start.y);
    if (chord <= kEps) {
        appendUnique(result, end);
        return result;
    }

    const double sweep = 4.0 * std::atan(bulge);
    const double halfSweep = std::abs(sweep) * 0.5;
    const double sinHalf = std::sin(halfSweep);

    if (std::abs(sinHalf) <= kEps) {
        appendUnique(result, end);
        return result;
    }

    const double radius = chord / (2.0 * sinHalf);
    const double offset =
        chord / (2.0 * std::tan(halfSweep));

    const Point midpoint{
        (start.x + end.x) * 0.5,
        (start.y + end.y) * 0.5
    };

    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double invChord = 1.0 / chord;
    const Point left{-dy * invChord, dx * invChord};

    const double sign = sweep >= 0.0 ? 1.0 : -1.0;
    const Point center{
        midpoint.x - sign * left.x * offset,
        midpoint.y - sign * left.y * offset
    };

    const double startAngle =
        std::atan2(start.y - center.y, start.x - center.x);

    appendArc(
        result,
        center,
        radius,
        startAngle,
        sweep,
        tolerance
    );

    if (!result.empty() && !samePoint(result.back(), end)) {
        result.push_back(end);
    }

    return result;
}

Polygon makeLwPolyline(const Entity& entity, bool& closed, double tolerance) {
    struct Vertex {
        Point point{};
        double bulge{};
    };

    std::vector<Vertex> vertices;
    Vertex current{};
    bool haveX = false;
    bool haveY = false;

    for (const auto& group : entity.groups) {
        if (group.code == 10) {
            if (haveX && haveY) {
                vertices.push_back(current);
            }
            current = {};
            current.point.x = toNumber(group.value);
            haveX = true;
            haveY = false;
        } else if (group.code == 20 && haveX) {
            current.point.y = toNumber(group.value);
            haveY = true;
        } else if (group.code == 42 && haveX) {
            current.bulge = toNumber(group.value);
        }
    }

    if (haveX && haveY) {
        vertices.push_back(current);
    }

    closed = (groupInt(entity, 70) & 1) != 0;

    if (vertices.size() < 2) {
        return {};
    }

    Polygon result;
    const std::size_t edgeCount = closed ? vertices.size() : vertices.size() - 1;

    for (std::size_t i = 0; i < edgeCount; ++i) {
        const auto& a = vertices[i];
        const auto& b = vertices[(i + 1) % vertices.size()];
        const auto edge = makeBulgeSegment(a.point, b.point, a.bulge, tolerance);

        for (std::size_t p = 0; p < edge.size(); ++p) {
            if (i > 0 && p == 0) {
                continue;
            }
            appendUnique(result, edge[p]);
        }
    }

    if (closed && !result.empty() && samePoint(result.front(), result.back())) {
        result.pop_back();
    }

    return result;
}

Polygon makeClassicPolyline(
    const Entity& polyline,
    const std::vector<Entity>& vertices,
    bool& closed,
    double tolerance
) {
    struct Vertex {
        Point point{};
        double bulge{};
    };

    std::vector<Vertex> data;
    for (const auto& entity : vertices) {
        if (entity.type != "VERTEX") {
            continue;
        }
        data.push_back({
            groupPoint(entity, 10, 20),
            groupValue(entity, 42, 0.0)
        });
    }

    closed = (groupInt(polyline, 70) & 1) != 0;

    if (data.size() < 2) {
        return {};
    }

    Polygon result;
    const std::size_t edgeCount = closed ? data.size() : data.size() - 1;

    for (std::size_t i = 0; i < edgeCount; ++i) {
        const auto& a = data[i];
        const auto& b = data[(i + 1) % data.size()];
        const auto edge = makeBulgeSegment(a.point, b.point, a.bulge, tolerance);

        for (std::size_t p = 0; p < edge.size(); ++p) {
            if (i > 0 && p == 0) {
                continue;
            }
            appendUnique(result, edge[p]);
        }
    }

    if (closed && !result.empty() && samePoint(result.front(), result.back())) {
        result.pop_back();
    }

    return result;
}

std::vector<Entity> consumeClassicPolyline(
    const std::vector<Entity>& entities,
    std::size_t& index,
    bool& closed,
    double tolerance
) {
    const Entity& polyline = entities[index];
    std::vector<Entity> vertices;

    ++index;
    while (index < entities.size() && entities[index].type != "SEQEND") {
        if (entities[index].type == "VERTEX") {
            vertices.push_back(entities[index]);
        }
        ++index;
    }

    if (index < entities.size() && entities[index].type == "SEQEND") {
        ++index;
    }

    return {Entity{"__POLYGON__", {}}}, makeClassicPolyline(polyline, vertices, closed, tolerance);
}

std::vector<Segment> collectSegments(
    const std::vector<Entity>& entities,
    std::vector<Loop>& directLoops,
    std::vector<DxfDiagnostic>& diagnostics,
    double tolerance
) {
    std::vector<Segment> segments;

    for (std::size_t i = 0; i < entities.size();) {
        const auto& entity = entities[i];

        if (entity.type == "LINE") {
            const Point start = groupPoint(entity, 10, 20);
            const Point end = groupPoint(entity, 11, 21);

            if (!samePoint(start, end)) {
                Segment s;
                s.points = {start, end};
                s.start = start;
                s.end = end;
                s.source = "LINE";
                segments.push_back(std::move(s));
            }
            ++i;
            continue;
        }

        if (entity.type == "ARC") {
            const Point center = groupPoint(entity, 10, 20);
            const double radius = groupValue(entity, 40);
            const double startDeg = groupValue(entity, 50);
            const double endDeg = groupValue(entity, 51);
            double sweepDeg = endDeg - startDeg;
            if (sweepDeg <= 0.0) {
                sweepDeg += 360.0;
            }

            Polygon arc;
            appendArc(
                arc,
                center,
                radius,
                startDeg * kPi / 180.0,
                sweepDeg * kPi / 180.0,
                tolerance
            );

            if (arc.size() >= 2) {
                segments.push_back({
                    arc,
                    arc.front(),
                    arc.back(),
                    "ARC",
                    false
                });
            }
            ++i;
            continue;
        }

        if (entity.type == "CIRCLE") {
            const auto polygon = makeCircle(
                groupPoint(entity, 10, 20),
                groupValue(entity, 40),
                tolerance
            );
            if (polygon.size() >= 3) {
                directLoops.push_back({polygon, "CIRCLE", signedArea(polygon)});
            }
            ++i;
            continue;
        }

        if (entity.type == "LWPOLYLINE") {
            bool closed = false;
            const auto polygon = makeLwPolyline(entity, closed, tolerance);
            if (closed && polygon.size() >= 3) {
                directLoops.push_back({
                    polygon,
                    "LWPOLYLINE",
                    signedArea(polygon)
                });
            } else if (!closed && polygon.size() >= 2) {
                diagnostics.push_back({
                    DxfSeverity::Warning,
                    "DXF/LWPOLYLINE",
                    "Открытая LWPOLYLINE пропущена: контур не замкнут."
                });
            }
            ++i;
            continue;
        }

        if (entity.type == "POLYLINE") {
            bool closed = false;
            std::vector<Entity> vertices;

            ++i;
            while (i < entities.size() && entities[i].type != "SEQEND") {
                if (entities[i].type == "VERTEX") {
                    vertices.push_back(entities[i]);
                }
                ++i;
            }

            if (i < entities.size() && entities[i].type == "SEQEND") {
                ++i;
            }

            const auto polygon =
                makeClassicPolyline(entity, vertices, closed, tolerance);

            if (closed && polygon.size() >= 3) {
                directLoops.push_back({
                    polygon,
                    "POLYLINE",
                    signedArea(polygon)
                });
            } else if (!closed && polygon.size() >= 2) {
                diagnostics.push_back({
                    DxfSeverity::Warning,
                    "DXF/POLYLINE",
                    "Открытая POLYLINE пропущена: контур не замкнут."
                });
            }
            continue;
        }

        ++i;
    }

    return segments;
}

bool pointInPolygonInclusive(const Point& point, const Polygon& polygon) {
    return pointInPolygon(point, polygon);
}

void normalizeLoopOrientation(Polygon& polygon, bool outer) {
    const double area = signedArea(polygon);
    if ((outer && area < 0.0) || (!outer && area > 0.0)) {
        std::reverse(polygon.begin(), polygon.end());
    }
}

std::vector<DxfContour> buildContours(
    std::vector<Loop> loops
) {
    std::vector<DxfContour> result;

    if (loops.empty()) {
        return result;
    }

    std::sort(
        loops.begin(),
        loops.end(),
        [](const Loop& a, const Loop& b) {
            return std::abs(a.signedArea) > std::abs(b.signedArea);
        }
    );

    struct OuterRef {
        std::size_t loopIndex{};
        std::size_t contourIndex{};
        double area{};
    };

    std::vector<OuterRef> outers;

    for (std::size_t i = 0; i < loops.size(); ++i) {
        const auto& loop = loops[i];
        const Point probe = loop.polygon.front();

        std::vector<OuterRef> parents;
        for (const auto& outer : outers) {
            if (pointInPolygonInclusive(
                    probe,
                    loops[outer.loopIndex].polygon)) {
                parents.push_back(outer);
            }
        }

        if (parents.empty()) {
            auto polygon = loop.polygon;
            normalizeLoopOrientation(polygon, true);
            result.push_back({
                std::move(polygon),
                {},
                loop.source + "-" + std::to_string(result.size() + 1)
            });
            outers.push_back({
                i,
                result.size() - 1,
                std::abs(loop.signedArea)
            });
            continue;
        }

        // A loop enclosed by an outer loop is a hole. If another outer
        // loop is nested inside that hole, it becomes a separate island.
        const auto nearest = *std::min_element(
            parents.begin(),
            parents.end(),
            [](const OuterRef& a, const OuterRef& b) {
                return a.area < b.area;
            }
        );

        auto hole = loop.polygon;
        normalizeLoopOrientation(hole, false);
        result[nearest.contourIndex].holes.push_back(std::move(hole));
    }

    return result;
}

std::vector<Loop> buildLineArcLoops(
    std::vector<Segment> segments,
    std::vector<DxfDiagnostic>& diagnostics
) {
    std::vector<Loop> loops;

    for (std::size_t startIndex = 0; startIndex < segments.size(); ++startIndex) {
        if (segments[startIndex].used) {
            continue;
        }

        auto& first = segments[startIndex];
        first.used = true;

        Polygon polygon = first.points;
        const Point initial = first.start;
        Point current = first.end;

        bool closed = false;
        const std::size_t guardLimit = segments.size() + 1;

        for (std::size_t guard = 0; guard < guardLimit; ++guard) {
            if (samePoint(current, initial)) {
                closed = true;
                break;
            }

            std::size_t nextIndex = segments.size();
            bool reverse = false;

            for (std::size_t i = 0; i < segments.size(); ++i) {
                if (segments[i].used) {
                    continue;
                }

                if (samePoint(segments[i].start, current)) {
                    nextIndex = i;
                    reverse = false;
                    break;
                }

                if (samePoint(segments[i].end, current)) {
                    nextIndex = i;
                    reverse = true;
                    break;
                }
            }

            if (nextIndex == segments.size()) {
                break;
            }

            auto& next = segments[nextIndex];
            next.used = true;

            if (!reverse) {
                for (std::size_t p = 1; p < next.points.size(); ++p) {
                    appendUnique(polygon, next.points[p]);
                }
                current = next.end;
            } else {
                for (std::size_t p = next.points.size(); p-- > 1;) {
                    appendUnique(polygon, next.points[p - 1]);
                }
                current = next.start;
            }
        }

        if (closed && polygon.size() >= 3) {
            loops.push_back({
                polygon,
                "LINE/ARC",
                signedArea(polygon)
            });
        } else {
            diagnostics.push_back({
                DxfSeverity::Warning,
                "DXF/CONNECT",
                "Незамкнутая цепочка LINE/ARC пропущена."
            });
        }
    }

    return loops;
}

} // namespace

bool DxfDocument::valid() const {
    return !contours.empty();
}

DxfDocument importDxf(const std::string& text, double arcToleranceMm) {
    DxfDocument document;

    if (text.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/INPUT",
            "DXF-текст пуст."
        });
        return document;
    }

    const auto entities = readEntities(text, document.entitiesRead);

    if (entities.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/ENTITIES",
            "В DXF не найден раздел ENTITIES с распознаваемыми объектами."
        });
        return document;
    }

    std::vector<Loop> directLoops;
    auto segments = collectSegments(
        entities,
        directLoops,
        document.diagnostics,
        std::max(0.01, arcToleranceMm)
    );

    auto lineArcLoops =
        buildLineArcLoops(std::move(segments), document.diagnostics);

    directLoops.insert(
        directLoops.end(),
        std::make_move_iterator(lineArcLoops.begin()),
        std::make_move_iterator(lineArcLoops.end())
    );

    document.closedLoopsFound = directLoops.size();
    document.contours = buildContours(std::move(directLoops));

    if (document.contours.empty()) {
        document.diagnostics.push_back({
            DxfSeverity::Error,
            "DXF/GEOMETRY",
            "Не найден ни один замкнутый геометрический контур."
        });
    } else {
        document.diagnostics.push_back({
            DxfSeverity::Info,
            "DXF/GEOMETRY",
            "DXF успешно преобразован в замкнутые внешние контуры и отверстия."
        });
    }

    return document;
}

} // namespace sheetnest
